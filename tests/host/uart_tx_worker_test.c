/** @file uart_tx_worker_test.c
 * @brief 捕获生产UART任务入口，验证创建失败、重试及有限循环；不增加生产测试入口。
 */
#include "bsp_usart.h"
#include "uart_tx_hal_model.h"
#include "cmsis_os.h"
#include "task_monitor.h"
#include "test_check.h"
#include <setjmp.h>
static osThreadDef_t captured;
static unsigned creates, monitor_begins, monitor_completions, monitor_waits, delays, notifications;
static uint8_t fail_create, kernel_running;
static jmp_buf finished;
int32_t osKernelRunning(void)
{
    return kernel_running;
}
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    TEST_CHECK(argument == NULL);
    captured = *definition;
    ++creates;
    return fail_create ? NULL : &captured;
}
TaskMonitor_Result_e TaskMonitorBeginWork(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == TASK_MONITOR_UART_TX);
    ++monitor_begins;
    return TASK_MONITOR_RESULT_OK;
}
TaskMonitor_Result_e TaskMonitorCompleteWork(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == TASK_MONITOR_UART_TX);
    ++monitor_completions;
    return TASK_MONITOR_RESULT_OK;
}
TaskMonitor_Result_e TaskMonitorSetWaiting(TaskMonitor_Id_e id, TaskMonitor_WaitReason_e reason)
{
    TEST_CHECK(id == TASK_MONITOR_UART_TX && reason == TASK_MONITOR_WAIT_PERIOD);
    ++monitor_waits;
    return TASK_MONITOR_RESULT_OK;
}
void osDelay(uint32_t ms)
{
    TEST_CHECK(ms == 1u);
    ++delays;
    if (delays == 1u)
    {
        UARTModelDMAComplete(0u);
        UARTModelComplete(0u);
    }
    if (delays == 2u)
        longjmp(finished, 1);
}
static void rx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    (void)block;
}
static void done(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    TEST_CHECK(event->result == USART_TX_COMPLETE);
    ++notifications;
}
int main(void)
{
    UARTModelInit();
    USART_Init_Config_s config = {
        .recv_buff_size = 1u, .usart_handle = &uart_handles[0], .module_callback = rx};
    USARTInstance *instance = USARTRegister(&config);
    TEST_CHECK(instance);
    UARTModelCheckRuntimeAllocation();
    uint8_t data = 0x68u;
    uint32_t id = 77u;
    USARTTxRequest_t request = {&data, 1u, USART_TRANSFER_DMA, 50u, done, NULL, NULL};
    kernel_running=1u;
    TEST_CHECK(!USARTTxTaskInit() && creates==0u);
    kernel_running=0u;
    mc02_test_faultmask=1u;
    TEST_CHECK(!USARTTxTaskInit() && creates==0u);
    mc02_test_faultmask=0u;
    mc02_test_ipsr=1u;
    TEST_CHECK(!USARTTxTaskInit() && creates==0u);
    mc02_test_ipsr=0u;
    // RobotInit在PRIMASK=1时创建任务；初始化必须保持这一启动状态。
    mc02_test_disable_irq();
    mc02_test_basepri=0x50u; // 真实FreeRTOS在调度前创建前序任务后保持BASEPRI屏蔽。
    fail_create = 1u;
    TEST_CHECK(!USARTTxTaskInit());
    TEST_CHECK(mc02_test_primask==1u && creates==1u);
    mc02_test_set_primask(0u);
    mc02_test_basepri=0u;
    TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_NOT_READY && id == 77u &&
               uart_model_calls == 0u);
    fail_create = 0u;
    mc02_test_disable_irq();
    mc02_test_basepri=0x50u;
    TEST_CHECK(USARTTxTaskInit() && creates == 2u && USARTTxTaskInit() && creates == 2u);
    TEST_CHECK(mc02_test_primask==1u && mc02_test_basepri==0x50u);
    mc02_test_set_primask(0u);
    mc02_test_basepri=0u;
    TEST_CHECK(captured.stack_size == 512u && captured.priority == osPriorityNormal &&
               uart_model_calls == 0u);
    TEST_CHECK(USARTSubmitTx(instance, &request, &id) == USART_SEND_ACCEPTED);
    if (setjmp(finished) == 0)
        captured.entry(NULL);
    TEST_CHECK(monitor_begins == 2u && monitor_completions == 2u && monitor_waits == 2u &&
               delays == 2u && notifications == 1u && uart_model_leases == 0u);
    return 0;
}
