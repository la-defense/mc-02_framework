/** @file bsp_log.h
 * @brief 任务监控顺序测试的日志替身，只观察安全关断后的关键故障事件。
 */
#ifndef MC02_TASK_MONITOR_TEST_LOG_H
#define MC02_TASK_MONITOR_TEST_LOG_H

#include "../fakes/bsp_log.h"

void MC02TestLogError(void);

#undef BSP_LOG_EVENT_1
#undef BSP_LOG_EVENT_2
#define BSP_LOG_EVENT_1(name, a) do { if ((name) == EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_02) MC02TestLogError(); } while (0)
#define BSP_LOG_EVENT_2(name, a, b) do { if ((name) == EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_01) MC02TestLogError(); } while (0)

#endif
