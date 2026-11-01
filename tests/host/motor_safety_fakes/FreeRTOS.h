/** @file FreeRTOS.h
 * @brief 记录主机电机安全测试中的任务临界区进入次数。
 * @note 不模拟任务切换或中断屏蔽，只检查提交 CAN 请求时是否仍在临界区。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_MOTOR_SAFETY_TEST_FREERTOS_H
#define MC02_MOTOR_SAFETY_TEST_FREERTOS_H

extern unsigned motor_safety_test_critical_depth;

#define taskENTER_CRITICAL() (++motor_safety_test_critical_depth)
#define taskEXIT_CRITICAL() (--motor_safety_test_critical_depth)

#endif
