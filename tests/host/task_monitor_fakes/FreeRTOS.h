/** @file FreeRTOS.h
 * @brief 任务监控主机测试的任务临界区替身；监控自身使用中断状态保存接口。
 */
#ifndef MC02_TASK_MONITOR_FAKE_FREERTOS_H
#define MC02_TASK_MONITOR_FAKE_FREERTOS_H
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
