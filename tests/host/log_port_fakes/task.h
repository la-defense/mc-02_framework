/** @file task.h
 * @brief 日志静态任务创建使用的主机替身声明。
 */
#ifndef MC02_LOG_PORT_FAKE_TASK_H
#define MC02_LOG_PORT_FAKE_TASK_H
#include "FreeRTOS.h"
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name, uint32_t depth,
                               void *argument, uint32_t priority, StackType_t *stack,
                               StaticTask_t *control);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskSuspend(TaskHandle_t handle);
#endif
