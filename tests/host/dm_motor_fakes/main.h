/** @file main.h
 * @brief 为 DM 主机测试提供 HAL 毫秒时钟声明。
 * @note 仅供测试编译生产 Daemon，不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_DM_MOTOR_TEST_MAIN_H
#define MC02_DM_MOTOR_TEST_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);
extern volatile uint32_t test_ipsr;

static inline void __DMB(void) {}
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __set_PRIMASK(uint32_t value) { (void)value; }
static inline uint32_t __get_IPSR(void) { return test_ipsr; }

#endif
