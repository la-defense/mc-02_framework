/** @file FreeRTOS.h
 * @brief UART集成测试用同一可嵌套互斥锁模拟任务临界区和中断屏蔽。
 */
#ifndef MC02_UART_TEST_FREERTOS_H
#define MC02_UART_TEST_FREERTOS_H
#include "main.h"
#define taskENTER_CRITICAL() UARTModelCriticalEnter()
#define taskEXIT_CRITICAL() UARTModelCriticalExit()
#endif
