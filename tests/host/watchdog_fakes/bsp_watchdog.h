#ifndef MC02_HOST_WATCHDOG_TEST_BSP_WATCHDOG_H
#define MC02_HOST_WATCHDOG_TEST_BSP_WATCHDOG_H

#include <stdint.h>

void BSP_WatchdogInit(uint32_t timeout_ms);
uint8_t BSP_WatchdogInitChecked(uint32_t timeout_ms);
uint8_t BSP_WatchdogSetTimeoutChecked(uint32_t timeout_ms);
uint8_t BSP_WatchdogIsRunning(void);
uint32_t BSP_WatchdogGetTimeout(void);
void BSP_WatchdogFeed(void);

#endif
