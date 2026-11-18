/** @file task.h
 * @brief 模拟 FreeRTOS 调度器状态及暂停／恢复调用。
 */
#ifndef MC02_TEST_TASK_BATCH_TASK_H
#define MC02_TEST_TASK_BATCH_TASK_H
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
#define taskSCHEDULER_SUSPENDED 2
BaseType_t xTaskGetSchedulerState(void);
void vTaskSuspendAll(void);
BaseType_t xTaskResumeAll(void);
TickType_t xTaskGetTickCount(void); // 读取由测试推进的计数。
void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment); // 记录期望的绝对时刻。
#endif
