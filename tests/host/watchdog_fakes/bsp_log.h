/** @file bsp_log.h
 * @brief 看门狗主机测试的日志替身，区分错误和初始化状态事件。
 */
#ifndef MC02_WATCHDOG_TEST_LOG_H
#define MC02_WATCHDOG_TEST_LOG_H

#include "../fakes/bsp_log.h"

void MC02TestLogError(void);
void MC02TestLogInfo(void);

#undef BSP_LOG_EVENT_0
#undef BSP_LOG_EVENT_1
#define BSP_LOG_EVENT_0(name) do { if ((name) == EV_BSP_WATCHDOG_BSP_WATCHDOG_C_01 || (name) == EV_BSP_WATCHDOG_BSP_WATCHDOG_C_02) MC02TestLogError(); } while (0)
#define BSP_LOG_EVENT_1(name, a) do { if ((name) == EV_BSP_WATCHDOG_BSP_WATCHDOG_C_03) MC02TestLogInfo(); } while (0)

#endif
