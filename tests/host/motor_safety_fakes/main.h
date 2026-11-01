/** @file main.h
 * @brief 为主机 Safety 测试提供中断屏蔽寄存器的替身。
 * @note 只保存测试用状态，不模拟真实 MCU 中断。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_MOTOR_SAFETY_TEST_MAIN_H
#define MC02_MOTOR_SAFETY_TEST_MAIN_H
#include <stdint.h>
static uint32_t mc02_motor_safety_test_primask;
#define __get_PRIMASK() (mc02_motor_safety_test_primask)
#define __disable_irq() (mc02_motor_safety_test_primask = 1u)
#define __set_PRIMASK(value) (mc02_motor_safety_test_primask = (value))
#endif
