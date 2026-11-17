/** @file main.h
 * @brief 测试平台同步后端只需模拟毫秒计数声明。
 */
#ifndef MC02_MOTOR_PID_TEST_MAIN_H
#define MC02_MOTOR_PID_TEST_MAIN_H
#include "stm32h7xx_hal.h"
// 单线程测试模拟原有中断屏蔽状态，验证保存和恢复可配对。
static inline uint32_t *MotorTestIrqState(void) { static uint32_t state; return &state; }
#define __get_PRIMASK() (*MotorTestIrqState())
#define __disable_irq() (*MotorTestIrqState() = 1u)
#define __set_PRIMASK(value) (*MotorTestIrqState() = (value))
#endif
