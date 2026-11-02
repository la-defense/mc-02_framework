/** @file protocol_uart_binding_fake.c
 * @brief 用测试指定的句柄模拟裁判串口绑定，不访问主机外设。
 */
#include "protocol_uart_binding_fake.h"
#include <stddef.h>

static UART_HandleTypeDef *referee_handle;
static UART_HandleTypeDef *vision_handle;

void TestRefereeUartBind(UART_HandleTypeDef *handle)
{
    referee_handle = handle;
}

void TestVisionUartBind(UART_HandleTypeDef *handle)
{
    vision_handle = handle;
}

UART_HandleTypeDef *BoardUartResolve(PlatformUartPort_t port)
{
    if (port == PLATFORM_UART_1)
        return referee_handle;
    return port == PLATFORM_UART_9 ? vision_handle : NULL;
}
