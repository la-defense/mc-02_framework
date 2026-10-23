/** @file bsp_hal_tick.h
 * @brief LCD 主机测试替身的 HAL Tick lease 接口。
 */
#ifndef MC02_LCD_HAL_TICK_FAKE_H
#define MC02_LCD_HAL_TICK_FAKE_H
#include <stdint.h>
uint8_t BSP_HALTickTryAcquireBlockingWait(void);
void BSP_HALTickReleaseBlockingWait(void);
#endif
