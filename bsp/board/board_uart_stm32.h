/** @file board_uart_stm32.h
 * @brief MC-02 逻辑 UART 与 STM32 串口的私有适配入口。
 */
#ifndef MC02_BOARD_UART_STM32_H
#define MC02_BOARD_UART_STM32_H
#include "platform_resources.h"
#include "usart.h"

/** @brief 根据逻辑编号查找 MC-02 的 UART 句柄。
 * @param[in] port 车型指定的逻辑 UART 编号。
 * @return 对应的板级句柄；未绑定编号返回 NULL。
 * @note 句柄存在不代表串口或外部设备已工作；仅供 STM32 适配代码使用。
 */
UART_HandleTypeDef *BoardUartResolve(PlatformUartPort_t port);
#endif
