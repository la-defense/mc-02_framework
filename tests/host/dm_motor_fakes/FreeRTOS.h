/** @file FreeRTOS.h
 * @brief 将 DM 主机测试的任务临界区映射到同步替身。
 */
#ifndef MC02_DM_MOTOR_TEST_FREERTOS_H
#define MC02_DM_MOTOR_TEST_FREERTOS_H

void dm_test_critical_enter(void);
void dm_test_critical_exit(void);
#define taskENTER_CRITICAL() dm_test_critical_enter()
#define taskEXIT_CRITICAL() dm_test_critical_exit()

#endif
