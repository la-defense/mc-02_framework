/**
 * @file bsp_dwt.h
 * @brief LK 主机回归使用的微秒时钟替身，不读取硬件计时器。
 */
#ifndef MC02_LK_TEST_DWT_H
#define MC02_LK_TEST_DWT_H
#include <stdint.h>
uint64_t DWT_GetTimeline_us(void);
float DWT_GetDeltaT(uint32_t *counter);
#endif
