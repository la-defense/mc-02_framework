#ifndef MC02_HOST_SPI_FAKE_DWT_H
#define MC02_HOST_SPI_FAKE_DWT_H
#include <stdint.h>
uint32_t DWT_ProbeStart(void);
uint32_t DWT_ProbeElapsedUs(uint32_t start);
#endif
