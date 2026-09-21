#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

void BSP_WatchdogInit(uint32_t timeout_ms);
void BSP_WatchdogFeed(void);
void BSP_WatchdogSetTimeout(uint32_t timeout_ms);
uint32_t BSP_WatchdogGetTimeout(void);
void BSP_WatchdogLogResetReason(void);
uint8_t BSP_WatchdogWasIwdgReset(void);
uint8_t BSP_WatchdogIsRunning(void);

#endif // !BSP_WATCHDOG_H
