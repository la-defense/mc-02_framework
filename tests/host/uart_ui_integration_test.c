/** @file uart_ui_integration_test.c
 * @brief 生产UI、CRC、裁判解析、UART注册/队列/H723后端的完整联测。
 * @note 仅HAL、RTOS、时间与daemon为模型；不连接硬件，不等待115ms。
 */
#include "protocol_uart_binding_fake.h"
#include "bsp_usart.h"
#include "referee_task.h"
#include "uart_tx_hal_model.h"
#include "test_check.h"
#include "crc.h"
#include "daemon.h"
#include <string.h>
// 序号由生产共享入口维护；测试通过只读副本查询。
static uint8_t sequence(void)
{
    RefereeTxState_t value;
    return RefereeGetTxState(&value) == USART_SEND_ACCEPTED ? value.next_sequence : 0u;
}
static DaemonInstance daemon_instance;
static uint16_t test_hp = 50u;
static const RefereeUIFieldConfig_t test_ui_fields[] = {
    {{'h', 'p', '0'}, "HP:", 120u, 195u, 20u, 2u, UI_Color_Green,
     REFEREE_UI_SOURCE_ROBOT_HP},
};
static const RefereeUIConfig_t test_ui_config = {
    .fields = test_ui_fields,
    .field_count = (uint8_t)(sizeof(test_ui_fields) / sizeof(test_ui_fields[0])),
    .frame_interval_ms = 115u,
};
static unsigned delays;
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    (void)config;
    return &daemon_instance;
}
uint8_t DaemonUnregister(DaemonInstance *instance)
{
    (void)instance;
    return 1u;
}
void DaemonReload(DaemonInstance *instance)
{
    (void)instance;
}
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{
    (void)instance;
    (void)received_ms;
    (void)flags;
    return DAEMON_OK;
}
uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    (void)instance;
    return 1u;
}
void osDelay(uint32_t ms)
{
    (void)ms;
    ++delays;
    TEST_CHECK(0);
}
static void identity(uint8_t robot)
{
    uint8_t frame[26] = {0xa5u, 17u, 0u, 0u, 0u, 1u, 2u};
    frame[7] = robot;
    frame[9] = (uint8_t)test_hp;
    frame[10] = (uint8_t)(test_hp >> 8u);
    frame[11] = 100u;
    uint32_t crc;
    TEST_CHECK(CRCCompute(CRC8_REFEREE, frame, 4u, &crc) == CRC_OK);
    frame[4] = (uint8_t)crc;
    TEST_CHECK(CRCCompute(CRC16_SP_REFEREE, frame, 24u, &crc) == CRC_OK);
    frame[24] = (uint8_t)crc;
    frame[25] = (uint8_t)(crc >> 8u);
    memcpy(uart_model_rx[0], frame, sizeof(frame));
    HAL_UARTEx_RxEventCallback(&uart_handles[0], sizeof(frame));
    USARTProcessRx();
}
static UIState_t state(void)
{
    UIState_t value;
    TEST_CHECK(UIGetState(&value) == UI_OK);
    return value;
}
static void complete(void)
{
    UARTModelDMAComplete(0u);
    UARTModelComplete(0u);
    USARTProcessTx();
}
static void ignored(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    (void)event;
}
static void advance(void)
{
    uart_model_ms += 115u;
    identity(1u);
}
static void init_ui(void)
{
    TEST_CHECK(MyUIInit() == UI_SUBMITTED);
    unsigned total = (unsigned)test_ui_config.field_count + 1u;
    for (unsigned i = 0u; i < total; ++i)
    {
        advance();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        unsigned previous = uart_model_calls;
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == previous + 1u);
        TEST_CHECK(state().phase == UI_PHASE_INITIALIZING && state().init_step == i);
        complete();
        TEST_CHECK(UIRefresh() == UI_OK);
    }
    TEST_CHECK(state().phase == UI_PHASE_RUNNING);
}
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    UARTModelInit();
    USARTTxSetServiceReady(1u);
    TestRefereeUartBind(&uart_handles[0]);
    TEST_CHECK(RefereeInit(PLATFORM_UART_1, NULL) == REFEREE_OK && UITaskInit(&test_ui_config) == UI_OK);
    identity(1u);
    UARTModelCheckRuntimeAllocation();
    if (strcmp(argv[1], "identity-before-start") == 0)
    {
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
        identity(2u);
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 0u && UIRefresh() == UI_SUBMITTED &&
                   state().phase == UI_PHASE_INITIALIZING && state().init_step == 0u);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 1u);
        return 0;
    }
    if (strcmp(argv[1], "old-generation") == 0)
    {
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
        uint32_t generation = state().generation;
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && state().generation != generation);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 0u);
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 1u && state().init_step == 0u);
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && state().init_step == 1u);
        return 0;
    }
    if (strcmp(argv[1], "queue-full") == 0)
    {
        uint8_t byte = 0u;
        uint32_t id;
        USARTTxRequest_t request = {&byte, 1u, USART_TRANSFER_DMA, 100u, ignored, NULL, NULL};
        // 不启动这些测试报文，先验证UI提交遇满不消耗序号或初始化步骤。
        USARTInstance *instance = NULL;
        extern USARTInstance *USARTRegisteredAt(unsigned index);
        instance = USARTRegisteredAt(0u);
        TEST_CHECK(instance);
        for (unsigned i = 0u; i < 4u; ++i)
            TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ACCEPTED);
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SEND_BUSY && sequence() == 0u &&
                   state().init_step == 0u);
        for (unsigned i = 0u; i < 5u; ++i)
            USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED && sequence() == 1u && uart_model_calls == 0u);
        USARTProcessTx();
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && state().init_step == 1u);
        return 0;
    }
    if (strcmp(argv[1], "partial-init") == 0 || strcmp(argv[1], "reinit-inflight") == 0)
    {
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        if (strcmp(argv[1], "reinit-inflight") == 0)
        {
            TEST_CHECK(MyUIInit() == UI_SUBMITTED);
            USARTProcessTx();
            TEST_CHECK(UIRefresh() == UI_OK && uart_model_aborts == 1u && state().init_step == 0u);
            advance();
            TEST_CHECK(UIRefresh() == UI_SUBMITTED);
            USARTProcessTx();
            complete();
            TEST_CHECK(UIRefresh() == UI_OK && state().init_step == 1u && uart_model_calls == 2u);
            return 0;
        }
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && state().init_step == 1u);
        advance();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        uart_model_start_result = HAL_ERROR;
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED && state().phase == UI_PHASE_ERROR);
        TEST_CHECK(state().init_step == 1u && sequence() == 2u);
        uart_model_start_result = HAL_OK;
        advance();
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED);
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED && state().init_step == 0u);
        return 0;
    }
    init_ui();
    if (strcmp(argv[1], "readonly") == 0)
    {
        unsigned previous = uart_model_calls;
        for (unsigned i = 0u; i < 100u; ++i)
        {
            ++uart_model_ms;
            identity(1u);
            TEST_CHECK(UITask() == UI_OK);
            USARTProcessTx();
        }
        TEST_CHECK(uart_model_calls == previous);
        test_hp = 60u;
        advance();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        TEST_CHECK(memcmp(uart_model_tx[0] + 28u, "HP:60/100", 9u) == 0);
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && uart_model_calls == previous + 1u);
        return 0;
    }
    if (strcmp(argv[1], "link-recovery") == 0)
    {
        uart_model_ms += 301u;
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED && state().phase == UI_PHASE_NEED_INIT);
        identity(1u);
        TEST_CHECK(UIRefresh() == UI_SUBMITTED && state().phase == UI_PHASE_INITIALIZING &&
                   state().init_step == 0u);
        USARTProcessTx();
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && state().init_step == 1u);
        return 0;
    }
    if (strcmp(argv[1], "cadence") == 0)
    {
        unsigned previous = uart_model_calls;
        test_hp = 60u;
        for (unsigned i = 0u; i < 100u; ++i)
        {
            ++uart_model_ms;
            identity(1u);
            (void)UIRefresh();
            USARTProcessTx();
        }
        TEST_CHECK(uart_model_calls == previous && delays == 0u && uart_model_leases == 0u);
        uart_model_ms += 15u;
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == previous + 1u);
        return 0;
    }
    test_hp = 60u;
    advance();
    TEST_CHECK(UIRefresh() == UI_SUBMITTED);
    USARTProcessTx();
    if (strcmp(argv[1], "failure") == 0)
    {
        uart_model_ms += 100u;
        USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED);
        TEST_CHECK(state().phase == UI_PHASE_ERROR);
        TEST_CHECK(uart_model_aborts == 1u && uart_model_leases == 0u);
        RefereeTxState_t tx;
        TEST_CHECK(RefereeGetTxState(&tx) == USART_SEND_ACCEPTED);
        TEST_CHECK(tx.uart.failed == 1u && tx.uart.last_event.result == USART_TX_EXPIRED);
        return 0;
    }
    TEST_CHECK(strcmp(argv[1], "latest") == 0);
    test_hp = 61u;
    identity(1u);
    TEST_CHECK(UIRefresh() == UI_OK);
    complete();
    TEST_CHECK(UIRefresh() == UI_OK);
    advance();
    TEST_CHECK(UIRefresh() == UI_SUBMITTED);
    USARTProcessTx();
    TEST_CHECK(memcmp(uart_model_tx[0] + 28u, "HP:61/100", 9u) == 0);
    complete();
    TEST_CHECK(UIRefresh() == UI_OK && delays == 0u);
    TEST_CHECK(uart_model_leases == 0u);
    return 0;
}
