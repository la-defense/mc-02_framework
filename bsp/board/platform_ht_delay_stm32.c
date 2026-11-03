/** @file platform_ht_delay_stm32.c
 * @brief 将 HT 电机既有初始化忙等交给 MC-02 DWT。
 */
#include "../dwt/platform_probe.h"
#include "bsp_dwt.h"

void PlatformHtLegacyDelayMs(uint32_t milliseconds)
{
    DWT_Delay((float)milliseconds / 1000.0f); // 保留既有忙等与毫秒到秒的换算。
}
