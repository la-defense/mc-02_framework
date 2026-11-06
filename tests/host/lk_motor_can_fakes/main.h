/**
 * @file main.h
 * @brief LK 与生产 CAN 集成回归使用的 HAL 临界区替身。
 */
#ifndef MC02_LK_CAN_TEST_MAIN_H
#define MC02_LK_CAN_TEST_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

extern volatile uint32_t test_ipsr;
extern uint32_t can_test_irq_mask, can_test_irq_sections;
void CANTestRestoreIrq(uint32_t value);

static inline void __DMB(void) {}
static inline uint32_t __get_PRIMASK(void) { return can_test_irq_mask; }
static inline void __disable_irq(void) { can_test_irq_mask = 1u; ++can_test_irq_sections; }
static inline void __enable_irq(void) { can_test_irq_mask = 0u; }
static inline void __set_PRIMASK(uint32_t value) { CANTestRestoreIrq(value); }
static inline uint32_t __get_IPSR(void) { return test_ipsr; }

#endif
