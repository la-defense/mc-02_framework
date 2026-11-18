/** @file task.h
 * @brief 提供测试控制的当前任务和调度状态。
 */
#ifndef MC02_TEST_PERIODIC_TIM5_TASK_H
#define MC02_TEST_PERIODIC_TIM5_TASK_H
#include "FreeRTOS.h"
#define taskSCHEDULER_RUNNING 1
BaseType_t xTaskGetSchedulerState(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
#endif
