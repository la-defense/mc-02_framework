/** @file platform_feature_startup.h
 * @brief 日志端口测试使用的诊断 UART 就绪查询声明。
 */
#pragma once
#include <stdint.h>
enum { PLATFORM_UART_10 = 10 };
uint8_t PlatformFeatureUartReady(int port);
