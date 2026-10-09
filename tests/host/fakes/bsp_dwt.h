#ifndef MC02_TEST_BSP_DWT_H
#define MC02_TEST_BSP_DWT_H

#include <stdint.h>
extern volatile uint32_t dwt_cpu_freq_mhz;

void DWT_Delay(float seconds);

#endif
