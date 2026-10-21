/** @file uart_tx_test.c
 * @brief 通过生产注册、复制提交和完成事件验证UART队列，普通主机进程运行。
 * @note 每场景独立进程；HAL、时间和同步使用硬件模型，Release仍执行检查。
 */
#include "bsp_usart.h"
#include "test_check.h"
#include <string.h>
#include <stdatomic.h>
#include <pthread.h>

static atomic_uint now_ms, calls, aborts, leases, received;
static HAL_StatusTypeDef start_result = HAL_OK, abort_result = HAL_OK;
static uint8_t *borrowed[DEVICE_USART_CNT];
static uint16_t lengths[DEVICE_USART_CNT];
static UART_HandleTypeDef handles[DEVICE_USART_CNT];
static USARTInstance *instances[DEVICE_USART_CNT];
static USARTTxEvent_t events[128];
static unsigned notified;
uint32_t HAL_GetTick(void)
{
    return atomic_load(&now_ms);
}
uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    atomic_fetch_add(&leases, 1u);
    return 1u;
}
void BSP_HALTickReleaseBlockingWait(void)
{
    TEST_CHECK(atomic_fetch_sub(&leases, 1u) > 0u);
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
    (void)h;
    (void)p;
    (void)n;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{
    (void)t;
    unsigned index = (unsigned)(h - handles);
    TEST_CHECK(index < DEVICE_USART_CNT);
    atomic_fetch_add(&calls, 1u);
    if (start_result == HAL_OK)
    {
        borrowed[index] = p;
        lengths[index] = n;
    }
    return start_result;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
    return HAL_UART_Transmit(h, p, n, 0u);
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
    return HAL_UART_Transmit(h, p, n, 0u);
}
HAL_StatusTypeDef UARTTestAbort(UART_HandleTypeDef *h)
{
    atomic_fetch_add(&aborts, 1u);
    if (abort_result == HAL_OK)
        h->gState = HAL_UART_STATE_READY;
    return abort_result;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
static void rx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    TEST_CHECK(block->length == 1u);
    atomic_fetch_add(&received, 1u);
}
static void done(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    TEST_CHECK(notified < 128u);
    events[notified++] = *event;
    USARTProcessTx(); // 回调递归推进不会重复通知或启动。
}
static void setup(unsigned n)
{
    static DMA_HandleTypeDef dmas[DEVICE_USART_CNT];
    for (unsigned i = 0; i < n; ++i)
    {
        handles[i].hdmarx = &dmas[i];
        handles[i].gState = HAL_UART_STATE_READY;
        USART_Init_Config_s c = {.recv_buff_size = 1u, .usart_handle = &handles[i], .module_callback = rx};
        instances[i] = USARTRegister(&c);
        TEST_CHECK(instances[i]);
    }
    USARTTxSetServiceReady(1u);
}
static uint32_t submit(unsigned port, const uint8_t *p, size_t n, uint32_t ttl)
{
    USARTTxRequest_t r = {
        .data = p, .length = n, .mode = USART_TRANSFER_DMA, .timeout_ms = ttl, .complete = done};
    uint32_t id = 0u;
    TEST_CHECK(USARTSubmitTx(instances[port], &r, &id) == USART_SEND_ACCEPTED && id != 0u);
    return id;
}
static void complete(unsigned port)
{
    handles[port].gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&handles[port]);
    USARTProcessTx();
}
static void *producer(void *arg)
{
    unsigned i = (unsigned)(uintptr_t)arg;
    uint8_t data[3] = {(uint8_t)i, 0x5au, 0xa5u};
    submit(0u, data, 3u, 50u);
    return NULL;
}
int main(int argc, char **argv)
{
    const char *scenario = argc > 1 ? argv[1] : "copy";
    setup(strcmp(scenario, "ports") == 0 ? DEVICE_USART_CNT : 1u);
    uint8_t data[128];
    memset(data, 0x39, sizeof(data));
    if (strcmp(scenario, "copy") == 0)
    {
        uint32_t id = submit(0, data, sizeof(data), 50u);
        memset(data, 0xee, sizeof(data));
        TEST_CHECK(calls == 0u);
        USARTProcessTx();
        TEST_CHECK(calls == 1u && lengths[0] == 128u);
        for (unsigned i = 0; i < 128u; ++i)
            TEST_CHECK(borrowed[0][i] == 0x39u);
        USARTProcessTx();
        TEST_CHECK(notified == 0u);
        complete(0u);
        TEST_CHECK(notified == 1u && events[0].id == id && events[0].result == USART_TX_COMPLETE &&
                   leases == 0u);
        complete(0u);
        TEST_CHECK(notified == 1u);
        return 0;
    }
    if (strcmp(scenario, "capacity") == 0 || strcmp(scenario, "threads") == 0)
    {
        if (strcmp(scenario, "threads") == 0)
        {
            pthread_t t[4];
            for (uintptr_t i = 0; i < 4u; ++i)
                TEST_CHECK(pthread_create(&t[i], NULL, producer, (void *)i) == 0);
            for (unsigned i = 0; i < 4u; ++i)
                TEST_CHECK(pthread_join(t[i], NULL) == 0);
        }
        else
            for (unsigned i = 0; i < 4u; ++i)
                submit(0, data, 1u, 50u);
        uint32_t untouched = 0xbad0u;
        USARTTxRequest_t r = {data, 1u, USART_TRANSFER_DMA, 50u, done, NULL, NULL};
        TEST_CHECK(USARTSubmitTx(instances[0], &r, &untouched) == USART_SEND_BUSY && untouched == 0xbad0u);
        USARTProcessTx();
        for (unsigned i = 0; i < 4u; ++i)
            complete(0u);
        TEST_CHECK(calls == 4u && notified == 4u && leases == 0u);
        return 0;
    }
    if (strcmp(scenario, "ports") == 0)
    {
        for (unsigned p = 0; p < DEVICE_USART_CNT; ++p)
            for (unsigned i = 0; i < 4u; ++i)
                submit(p, data, 1u, 50u);
        USARTProcessTx();
        TEST_CHECK(calls == DEVICE_USART_CNT);
        for (unsigned i = 0; i < 4u; ++i)
            for (unsigned p = 0; p < DEVICE_USART_CNT; ++p)
                complete(p);
        TEST_CHECK(notified == DEVICE_USART_CNT * 4u && leases == 0u);
        return 0;
    }
    if (strcmp(scenario, "busy") == 0)
    {
        start_result = HAL_BUSY;
        submit(0, data, 1u, 50u);
        USARTProcessTx();
        TEST_CHECK(notified == 0u);
        now_ms = 49u;
        USARTProcessTx();
        TEST_CHECK(notified == 0u && aborts == 0u);
        now_ms = 50u;
        USARTProcessTx();
        TEST_CHECK(events[0].result == USART_TX_EXPIRED && aborts == 0u && leases == 0u);
        return 0;
    }
    if (strcmp(scenario, "error") == 0)
    {
        start_result = HAL_ERROR;
        submit(0, data, 1u, 50u);
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(notified == 1u && events[0].result == USART_TX_FAILED && calls == 1u && leases == 0u);
        return 0;
    }
    if (strcmp(scenario, "timeout") == 0 || strcmp(scenario, "isolate") == 0 || strcmp(scenario, "wrap") == 0)
    {
        uint32_t base = strcmp(scenario, "wrap") == 0 ? UINT32_MAX - 20u : 0u;
        now_ms = base;
        submit(0, data, 1u, 50u);
        USARTProcessTx();
        now_ms = base + 49u;
        USARTProcessTx();
        TEST_CHECK(aborts == 0u);
        if (strcmp(scenario, "isolate") == 0)
            abort_result = HAL_ERROR;
        now_ms = base + 50u;
        USARTProcessTx();
        TEST_CHECK(aborts == 1u && notified == 1u);
        TEST_CHECK(events[0].result == (abort_result == HAL_OK ? USART_TX_EXPIRED : USART_TX_ISOLATED));
        TEST_CHECK(leases == (abort_result == HAL_OK ? 0u : 1u));
        complete(0u);
        TEST_CHECK(notified == 1u);
        return 0;
    }
    if (strcmp(scenario, "cancel") == 0)
    {
        uint32_t a = submit(0, data, 1u, 50u), b = submit(0, data, 1u, 50u);
        USARTProcessTx();
        TEST_CHECK(USARTCancelTx(instances[0], b) == USART_SEND_ACCEPTED);
        TEST_CHECK(USARTCancelTx(instances[0], a) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(notified == 2u && aborts == 1u && calls == 1u && leases == 0u);
        TEST_CHECK(events[0].result == USART_TX_CANCELLED && events[1].result == USART_TX_CANCELLED);
        return 0;
    }
    if (strcmp(scenario, "rx") == 0)
    {
        submit(0, data, 1u, 50u);
        USARTProcessTx();
        now_ms = 50u;
        USARTProcessTx();
        instances[0]->recv_buff[0] = 0x44u;
        HAL_UARTEx_RxEventCallback(&handles[0], 1u);
        USARTProcessRx();
        TEST_CHECK(received == 1u && events[0].result == USART_TX_EXPIRED);
        return 0;
    }
    TEST_CHECK(0);
    return 1;
}
