/** @file uart_tx_backend_test.c
 * @brief 生产UART注册、队列和H723接入联测；只模拟HAL/寄存器和时钟。
 */
#include "bsp_usart.h"
#include "usart_tx_private.h"
#include "uart_tx_hal_model.h"
#include "test_check.h"
#include <string.h>
#include <pthread.h>
static USARTInstance *instance;
static USARTTxEvent_t event;
static unsigned notifications;
static uint8_t chain;
static void rx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    (void)block;
}
static void done(void *context, const USARTTxEvent_t *value)
{
    (void)context;
    TEST_CHECK(mc02_test_ipsr == 0u && mc02_test_primask == 0u);
    event = *value;
    ++notifications;
    if (chain)
    {
        chain = 0u;
        uint8_t bytes[2] = {0xc3u, 0x3cu};
        uint32_t id;
        USARTTxRequest_t request = {bytes, 2u, USART_TRANSFER_IT, 50u, done, NULL, NULL};
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ACCEPTED);
    }
}
static uint8_t deny(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    (void)data;
    (void)length;
    return 0u;
}
static void *cancel_thread(void *context)
{
    TEST_CHECK(USARTCancelTx(instance, *(uint32_t *)context) == USART_SEND_ACCEPTED);
    return NULL;
}
static void *timeout_thread(void *context)
{
    (void)context;
    uart_model_ms = 50u;
    USARTProcessTx();
    return NULL;
}
static void *complete_thread(void *context)
{
    (void)context;
    UARTModelDMAComplete(0u);
    UARTModelComplete(0u);
    return NULL;
}
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    UARTModelInit();
    TEST_CHECK(USARTTxNextId(0u) == 1u && USARTTxNextId(UINT32_MAX) == 1u &&
               USARTTxNextId(UINT32_MAX - 1u) == UINT32_MAX);
    USART_Init_Config_s config = {
        .recv_buff_size = 1u, .usart_handle = &uart_handles[0], .module_callback = rx};
    instance = USARTRegister(&config);
    TEST_CHECK(instance);
    config.usart_handle = &uart_handles[1];
    USARTInstance *other = USARTRegister(&config);
    TEST_CHECK(other);
    UARTModelCheckRuntimeAllocation();
    uint8_t bytes[129];
    memset(bytes, 0x96, sizeof(bytes));
    uint32_t id = 0x1234u;
    USARTTxRequest_t request = {bytes, 128u, USART_TRANSFER_DMA, 50u, done, NULL, NULL};
    TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_NOT_READY && id == 0x1234u);
    USARTTxSetServiceReady(1u);
    if (strcmp(argv[1], "arguments") == 0)
    {
        TEST_CHECK(USARTSubmitTx(NULL, &request, &id) == USART_SEND_BAD_ARGUMENT);
        TEST_CHECK(USARTSubmitTx(instance, NULL, &id) == USART_SEND_BAD_ARGUMENT);
        TEST_CHECK(USARTSubmitTx(instance, &request, NULL) == USART_SEND_BAD_ARGUMENT);
        request.length = 0u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.length = 129u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.length = 1u;
        request.timeout_ms = 0u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.timeout_ms = UINT32_MAX;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.timeout_ms = 50u;
        request.complete = NULL;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.complete = done;
        request.data = NULL;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.data = bytes;
        request.mode = USART_TRANSFER_BLOCKING;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        request.mode = USART_TRANSFER_DMA;
        mc02_test_ipsr = 1u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_BAD_ARGUMENT);
        mc02_test_ipsr = 0u;
        uart_model_irq_enabled[USART1_IRQn] = 0u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_model_irq_enabled[USART1_IRQn] = 1u;
        uart_tx_dma[0].Init.MemDataAlignment = 7u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_tx_dma[0].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        uart_handles[0].hdmatx = NULL;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_handles[0].hdmatx = &uart_tx_dma[0];
        // 分别破坏后端配置，拒绝不能借出缓冲或消耗编号。
        uint32_t *fields[] = {&uart_tx_dma[0].Init.Mode, &uart_tx_dma[0].Init.Direction,
                              &uart_tx_dma[0].Init.MemInc, &uart_tx_dma[0].Init.PeriphInc,
                              &uart_tx_dma[0].Init.PeriphDataAlignment,
                              &uart_handles[0].Init.Mode, &uart_handles[0].Init.WordLength,
                              &uart_handles[0].Init.Parity};
        for (unsigned i = 0u; i < sizeof(fields) / sizeof(fields[0]); ++i)
        {
            uint32_t previous = *fields[i];
            *fields[i] = i == 5u ? 0u : 0xffffffffu;
            TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
            *fields[i] = previous;
        }
        uart_tx_dma[0].Parent = NULL;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_tx_dma[0].Parent = &uart_handles[0];
        uart_model_irq_enabled[DMA1_Stream0_IRQn] = 0u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_model_irq_enabled[DMA1_Stream0_IRQn] = 1u;
        uart_handles[0].Instance->CR1 &= ~USART_CR1_TE;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ERROR);
        uart_handles[0].Instance->CR1 |= USART_CR1_TE;
        uart_model_lease_denied = 1u;
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_NOT_READY);
        TEST_CHECK(id == 0x1234u && uart_model_calls == 0u && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "guard") == 0)
        request.guard = deny;
    TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ACCEPTED);
    memset(bytes, 0xee, sizeof(bytes));
    USARTProcessTx();
    if (strcmp(argv[1], "guard") == 0)
    {
        USARTProcessTx();
        TEST_CHECK(event.result == USART_TX_CANCELLED && uart_model_calls == 0u && uart_model_leases == 0u);
        return 0;
    }
    TEST_CHECK(uart_model_calls == 1u && uart_model_tx[0][0] == 0x96u);
    if (strcmp(argv[1], "bounded") == 0)
    {
        // 两路同时到期，一轮只能停止一路；另一条仍持有缓冲和计时许可。
        TEST_CHECK(USARTSubmitTx(other, &request, &id) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        uart_model_ms = 50u;
        USARTProcessTx();
        TEST_CHECK(uart_model_aborts == 1u && notifications == 1u && uart_model_leases == 1u);
        USARTProcessTx();
        TEST_CHECK(uart_model_aborts == 2u && notifications == 2u && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "completion") == 0)
    {
        UARTModelDMAComplete(0u);
        USARTProcessTx();
        TEST_CHECK(notifications == 0u);
        chain = 1u;
        UARTModelComplete(0u);
        USARTProcessTx();
        TEST_CHECK(notifications == 1u && event.result == USART_TX_COMPLETE);
        TEST_CHECK(uart_model_calls == 2u && uart_model_tx[0][0] == 0xc3u);
        UARTModelComplete(0u);
        USARTProcessTx();
        TEST_CHECK(notifications == 2u && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "deadline-race") == 0)
    {
        uart_model_ms = 49u;
        pthread_t a, b;
        TEST_CHECK(pthread_create(&a, NULL, timeout_thread, NULL) == 0);
        TEST_CHECK(pthread_create(&b, NULL, complete_thread, NULL) == 0);
        TEST_CHECK(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
        USARTProcessTx();
        TEST_CHECK(notifications == 1u &&
                   (event.result == USART_TX_COMPLETE || event.result == USART_TX_EXPIRED));
        TEST_CHECK(uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "race") == 0)
    {
        pthread_t a, b;
        TEST_CHECK(pthread_create(&a, NULL, cancel_thread, &id) == 0);
        TEST_CHECK(pthread_create(&b, NULL, complete_thread, NULL) == 0);
        TEST_CHECK(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
        USARTProcessTx();
        TEST_CHECK(notifications == 1u && event.result == USART_TX_CANCELLED && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "error") == 0)
    {
        HAL_UART_ErrorCallback(&uart_handles[0]);
        USARTProcessTx();
        TEST_CHECK(event.result == USART_TX_FAILED && uart_model_aborts == 1u && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "isolate") == 0)
        uart_model_abort_result = HAL_TIMEOUT;
    if (strcmp(argv[1], "lost-dmat") == 0)
        uart_handles[0].Instance->CR3 &= ~USART_CR3_DMAT;
    // 接收标志保留不变，只清发送侧；到期回调不得再发布成功。
    uart_handles[0].Instance->CR3 |= 0x100u;
    uart_model_ms = 50u;
    HAL_UART_TxCpltCallback(&uart_handles[0]);
    USARTProcessTx();
    TEST_CHECK(uart_model_aborts == 1u && notifications == 1u && (uart_handles[0].Instance->CR3 & 0x100u));
    TEST_CHECK(event.result == (uart_model_abort_result == HAL_OK ? USART_TX_EXPIRED : USART_TX_ISOLATED));
    USARTTxState_t state;
    TEST_CHECK(USARTGetTxState(instance, &state) == USART_SEND_ACCEPTED);
    TEST_CHECK(state.isolated == (uart_model_abort_result == HAL_TIMEOUT));
    if (state.isolated)
    {
        TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ISOLATED && uart_model_leases == 1u);
        TEST_CHECK(USARTSubmitTx(other, &request, &id) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        UARTModelDMAComplete(1u);
        UARTModelComplete(1u);
        USARTProcessTx();
        TEST_CHECK(event.result == USART_TX_COMPLETE && notifications == 2u && uart_model_leases == 1u);
    }
    else
        TEST_CHECK(uart_model_leases == 0u);
    return 0;
}
