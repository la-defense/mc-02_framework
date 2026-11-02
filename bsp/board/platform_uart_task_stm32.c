/** @file platform_uart_task_stm32.c
 * @brief 将框架串口发送任务入口映射到现有 UART 服务。
 */
#include "../usart/platform_uart.h"
#include "board_uart_stm32.h"
#include "bsp_usart.h"

uint8_t PlatformUartStartTxTask(void)
{
    return USARTTxTaskInit();
}
