#ifndef MC02_HOST_BSP_HAL_TICK_H
#define MC02_HOST_BSP_HAL_TICK_H

#include <stdint.h>

uint8_t BSP_HALTickTryAcquireBlockingWait(void);
void BSP_HALTickReleaseBlockingWait(void);

#endif
