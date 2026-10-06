#ifndef BSP_HAL_TICK_H
#define BSP_HAL_TICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reserve HAL tick progress for a bounded HAL operation that uses HAL_GetTick().
   A suspended tick rejects new leases; an active lease defers HAL_SuspendTick(). */
uint8_t BSP_HALTickTryAcquireBlockingWait(void);
void BSP_HALTickReleaseBlockingWait(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_HAL_TICK_H */
