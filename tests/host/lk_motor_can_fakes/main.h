/**
 * @file main.h
 * @brief LK 与生产 CAN 集成回归使用的 HAL 临界区替身。
 */
#ifndef MC02_LK_CAN_TEST_MAIN_H
#define MC02_LK_CAN_TEST_MAIN_H

#include <stdint.h>

static inline void __DMB(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }

#endif
