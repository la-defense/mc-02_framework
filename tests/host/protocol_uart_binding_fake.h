/** @file protocol_uart_binding_fake.h
 * @brief 主机协议测试自行绑定逻辑串口和测试句柄。
 */
#ifndef MC02_PROTOCOL_UART_BINDING_FAKE_H
#define MC02_PROTOCOL_UART_BINDING_FAKE_H
#include "../../bsp/board/board_uart_stm32.h"

/** @brief 将测试句柄绑定至串口 1，句柄须活到测试结束。 */
void TestRefereeUartBind(UART_HandleTypeDef *handle);
/** @brief 将测试句柄绑定至串口 9，句柄须活到本次测试结束。 */
void TestVisionUartBind(UART_HandleTypeDef *handle);
#endif
