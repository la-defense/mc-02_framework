#ifndef BSP_CRASH_GUARD_H
#define BSP_CRASH_GUARD_H

#include <stdint.h>

/**
 * @brief Claim the crash-recording path or immediately request a system reset.
 * @param active Persistent only for the current boot; initialized to zero by startup.
 */
void CrashLogGuardEnter(volatile uint32_t *active);

#endif
