/** @file board_uart_stm32.c
 * @brief MC-02 逻辑 UART 编号与 H723 外设句柄的静态绑定。
 */
#include "board_uart_stm32.h"
#include <stddef.h>

UART_HandleTypeDef *BoardUartResolve(PlatformUartPort_t port)
{
    switch (port)
    {
    case PLATFORM_UART_1: return &huart1; // 逻辑串口 1 使用 USART1。
    case PLATFORM_UART_2: return &huart2; // 逻辑串口 2 使用 USART2。
    case PLATFORM_UART_3: return &huart3; // 逻辑串口 3 使用 USART3。
    case PLATFORM_UART_5: return &huart5; // 逻辑串口 5 使用 UART5。
    case PLATFORM_UART_7: return &huart7; // 逻辑串口 7 使用 UART7。
    case PLATFORM_UART_8: return &huart8; // 逻辑串口 8 使用 UART8。
    case PLATFORM_UART_9: return &huart9; // 逻辑串口 9 使用 UART9。
    case PLATFORM_UART_10: return &huart10; // 逻辑串口 10 使用 USART10。
    default: return NULL; // 未绑定的编号不映射到其他串口。
    }
}
