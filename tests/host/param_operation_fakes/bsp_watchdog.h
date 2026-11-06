#ifndef MC02_PARAM_OPERATION_FAKE_WATCHDOG_H
#define MC02_PARAM_OPERATION_FAKE_WATCHDOG_H

#include <stdint.h>

uint32_t BSP_WatchdogGetTimeout(void);
uint8_t BSP_WatchdogSetTimeoutChecked(uint32_t timeout_ms);
void BSP_WatchdogFeed(void);

#endif
