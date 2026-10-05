#ifndef MC02_HOST_BMI088_FAKE_CMSIS_OS_H
#define MC02_HOST_BMI088_FAKE_CMSIS_OS_H

#include <stdint.h>
uint8_t osKernelRunning(void);
void osDelay(uint32_t milliseconds);

#endif
