/** @file usart_tx_task.c
 * @brief 启动前创建唯一UART发送任务；创建失败保持提交不可用，接收登记不回滚。
 */
#include "usart_tx_private.h"
#include "cmsis_os.h"
#include "task_monitor.h"
static osThreadId uart_tx_task;
/** @brief 周期处理发送及通知，只有此任务调用USARTProcessTx。 */
static void UARTTxTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        TaskMonitorFeed(TASK_MONITOR_UART_TX);
        USARTProcessTx();
        osDelay(1u);
    }
}
uint8_t USARTTxTaskInit(void)
{
    if (!USARTTxTaskContext())
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
