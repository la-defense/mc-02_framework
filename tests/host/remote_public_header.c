/** @file remote_public_header.c
 * @brief 检查遥控公开头可不依赖芯片或 RTOS 头单独编译。
 */
#include "remote_control.h"

/** @brief 返回可用于车型配置的遥控逻辑串口编号。
 * @return MC-02 当前遥控使用的逻辑串口 5。
 */
PlatformUartPort_t RemotePublicHeaderProbe(void)
{
    return PLATFORM_UART_5;
}
