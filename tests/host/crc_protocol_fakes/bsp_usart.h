/** @file bsp_usart.h
 * @brief 协议编码测试的硬件对象模型；发送接口直接复用生产声明，避免签名漂移。
 */
#ifndef MC02_CRC_TEST_USART_H
#define MC02_CRC_TEST_USART_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
#include "../../../bsp/usart/bsp_usart.h"
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
