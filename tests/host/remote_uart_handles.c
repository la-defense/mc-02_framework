/** @file remote_uart_handles.c
 * @brief 为遥控主机测试提供其余板级 UART 句柄；UART5／9 由各测试单独保存。
 */
#include "main.h"
UART_HandleTypeDef huart1, huart2, huart3, huart7, huart8, huart10;
