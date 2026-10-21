/** @file uart_ui_integration_test.c
 * @brief 生产UI、CRC、裁判解析、UART注册/队列/H723后端的完整联测。
 * @note 仅HAL、RTOS、时间与daemon为模型；不连接硬件，不等待115ms。
 */
#include "referee_task.h"
#include "uart_tx_hal_model.h"
#include "test_check.h"
#include "crc.h"
#include "daemon.h"
#include <string.h>
static DaemonInstance daemon_instance;
static Referee_Interactive_info_t data;
static unsigned delays;
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
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
    for (unsigned i = 0u; i < 15u; ++i)
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
    TEST_CHECK(RefereeInit(&uart_handles[0], NULL) == REFEREE_OK && UITaskInit(&data) == UI_OK);
    identity(1u);
    UARTModelCheckRuntimeAllocation();
    if (strcmp(argv[1], "identity-before-start") == 0)
    {
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
        identity(2u);
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 0u && UIRefresh() == UI_NOT_INITIALIZED &&
                   state().phase == UI_PHASE_NEED_INIT);
        identity(1u);
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED && uart_model_calls == 0u);
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
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
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SEND_BUSY && UI_Seq == 0u &&
                   state().init_step == 0u);
        for (unsigned i = 0u; i < 5u; ++i)
            USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED && UI_Seq == 1u && uart_model_calls == 0u);
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
        TEST_CHECK(state().init_step == 1u && UI_Seq == 2u);
        uart_model_start_result = HAL_OK;
        advance();
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED);
        TEST_CHECK(MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED && state().init_step == 0u);
        return 0;
    }
    init_ui();
    if (strcmp(argv[1], "demo") == 0)
    {
        TEST_CHECK(UITask() == UI_OK);
        for (unsigned i = 0u; i < 114u; ++i)
        {
            ++uart_model_ms;
            (void)UITask();
        }
        TEST_CHECK(data.Chassis_Power_Data.chassis_power_mx == 0.0f);
        ++uart_model_ms;
        (void)UITask();
        TEST_CHECK(data.Chassis_Power_Data.chassis_power_mx == 3.5f);
        for (unsigned i = 0u; i < 100u; ++i)
            (void)UITask();
        TEST_CHECK(data.Chassis_Power_Data.chassis_power_mx == 3.5f);
        return 0;
    }
    if (strcmp(argv[1], "cadence") == 0)
    {
        unsigned previous = uart_model_calls;
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
    for (unsigned i = 0u; i < 6u; ++i)
    {
        advance();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        complete();
        TEST_CHECK(UIRefresh() == UI_OK);
    }
    data.chassis_mode = CHASSIS_ROTATE;
    advance();
    TEST_CHECK(UIRefresh() == UI_SUBMITTED);
    USARTProcessTx();
    if (strcmp(argv[1], "failure") == 0)
    {
        uart_model_ms += 100u;
        USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_NOT_INITIALIZED);
        TEST_CHECK(state().phase == UI_PHASE_ERROR && data.Referee_Interactive_Flag.chassis_flag);
        TEST_CHECK(uart_model_aborts == 1u && uart_model_leases == 0u);
        USARTTxState_t tx;
        TEST_CHECK(RefereeGetTxState(&tx) == USART_SEND_ACCEPTED);
        TEST_CHECK(tx.failed == 1u && tx.last_event.result == USART_TX_EXPIRED);
        return 0;
    }
    data.chassis_mode = CHASSIS_NO_FOLLOW;
    TEST_CHECK(UIRefresh() == UI_OK);
    complete();
    TEST_CHECK(UIRefresh() == UI_OK && data.Referee_Interactive_Flag.chassis_flag);
    advance();
    TEST_CHECK(UIRefresh() == UI_SUBMITTED);
    USARTProcessTx();
    complete();
    TEST_CHECK(UIRefresh() == UI_OK && !data.Referee_Interactive_Flag.chassis_flag && delays == 0u);
    TEST_CHECK(uart_model_leases == 0u);
    return 0;
}
