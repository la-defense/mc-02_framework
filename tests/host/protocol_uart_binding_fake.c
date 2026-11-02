/** @file protocol_uart_binding_fake.c
 * @brief 用测试指定的句柄模拟裁判串口绑定，不访问主机外设。
 */
#include "protocol_uart_binding_fake.h"
#include <stddef.h>

static UART_HandleTypeDef *referee_handle;

void TestRefereeUartBind(UART_HandleTypeDef *handle)
{
    referee_handle = handle;
}

UART_HandleTypeDef *BoardUartResolve(PlatformUartPort_t port)
{
    return port == PLATFORM_UART_1 ? referee_handle : NULL;
}
