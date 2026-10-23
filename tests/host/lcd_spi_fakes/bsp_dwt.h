/** @file bsp_dwt.h
 * @brief LCD 主机测试替身提供的单调探针接口。
 */
#ifndef MC02_LCD_DWT_FAKE_H
#define MC02_LCD_DWT_FAKE_H
#include <stdint.h>
uint32_t DWT_ProbeStart(void);
uint32_t DWT_ProbeElapsedUs(uint32_t start);
#endif
