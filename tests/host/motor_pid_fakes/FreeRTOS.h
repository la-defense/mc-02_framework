/** @file FreeRTOS.h
 * @brief 主机 PID 集成测试中的临界区替身。
 * @note 测试为单线程确定性交错，不模拟真实调度器临界区。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_MOTOR_PID_TEST_FREERTOS_H
#define MC02_MOTOR_PID_TEST_FREERTOS_H
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
