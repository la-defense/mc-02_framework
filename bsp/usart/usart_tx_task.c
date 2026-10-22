/** @file usart_tx_task.c
 * @brief 启动前创建唯一UART发送任务；创建失败保持提交不可用，接收登记不回滚。
 */
#include "usart_tx_private.h"
#include "cmsis_os.h"
#include "task_monitor.h"
#include "bsp_dwt.h"
static osThreadId uart_tx_task;
/* 记录发送任务每轮取队列和提交 HAL 的时间，查询由只读快照完成。 */
volatile DWT_Probe_t usart_tx_task_probe = {0};
/** @brief 周期处理发送及通知，只有此任务调用USARTProcessTx。 */
static void UARTTxTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        uint32_t probe = DWT_ProbeStart();
        TaskMonitorFeed(TASK_MONITOR_UART_TX);
        USARTProcessTx();
        DWT_ProbeDone(&usart_tx_task_probe, probe);
        osDelay(1u);
    }
}
uint8_t USARTTxTaskInit(void)
{
    // RobotInit在调度前保持PRIMASK=1；FreeRTOS创建前序任务后也可能保持BASEPRI。
    // 运行期提交仍要求普通任务且中断开启；这里不复用该运行检查。
    if (__get_IPSR()!=0u || __get_FAULTMASK()!=0u)
        return 0u;
    if (uart_tx_task != NULL)
        return 1u;
    if (osKernelRunning())
        return 0u;
    osThreadDef(uarttxtask, UARTTxTask, osPriorityNormal, 0, 512);
    uart_tx_task = osThreadCreate(osThread(uarttxtask), NULL);
    USARTTxSetServiceReady((uint8_t)(uart_tx_task != NULL));
    return (uint8_t)(uart_tx_task != NULL);
}
