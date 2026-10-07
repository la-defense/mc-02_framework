#ifndef MC02_TEST_CMSIS_OS_H
#define MC02_TEST_CMSIS_OS_H

#include <stdint.h>

uint8_t osKernelRunning(void);
void osDelay(uint32_t milliseconds);

#endif
