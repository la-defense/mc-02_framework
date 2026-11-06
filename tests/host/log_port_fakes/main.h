/** @file main.h
 * @brief 为日志端口提供最小芯片寄存器和串口类型。
 */
#ifndef MC02_LOG_PORT_FAKE_MAIN_H
#define MC02_LOG_PORT_FAKE_MAIN_H
#include <stdint.h>
typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef struct { uint32_t CCR; } SCB_Type;
extern SCB_Type log_test_scb;
extern uint32_t log_test_ipsr;
#define SCB (&log_test_scb)
#define SCB_CCR_DC_Msk 1u
#define __get_IPSR() (log_test_ipsr)
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
#endif
