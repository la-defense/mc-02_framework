#ifndef MC02_HOST_BMI088_FAKE_BSP_DWT_H
#define MC02_HOST_BMI088_FAKE_BSP_DWT_H

#include <stdint.h>
uint32_t DWT_ProbeStart(void);
uint32_t DWT_ProbeElapsedUs(uint32_t start);
void DWT_Delay(float seconds);
float DWT_GetTimeline_s(void);

#endif
