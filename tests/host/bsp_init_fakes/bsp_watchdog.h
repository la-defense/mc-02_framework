/** @file bsp_watchdog.h
 * @brief 主机启动测试的看门狗替身声明；超时单位为 ms，不启动硬件。
 */
#ifndef BSP_INIT_TEST_WATCHDOG_H
#define BSP_INIT_TEST_WATCHDOG_H
#include <stdint.h>
void BSP_WatchdogInit(uint32_t timeout_ms);
void BSP_WatchdogLogResetReason(void);
#endif
