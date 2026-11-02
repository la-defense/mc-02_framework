/** @file platform_probe_elapsed_stm32.c
 * @brief 将短时耗时查询绑定到现有 STM32 DWT 实现。
 */
#include "../dwt/platform_probe.h"
#include "bsp_dwt.h"

uint32_t PlatformProbeElapsedUs(uint32_t start)
{
    return DWT_ProbeElapsedUs(start); // 保持原有周期差、频率换算与回绕处理。
}
