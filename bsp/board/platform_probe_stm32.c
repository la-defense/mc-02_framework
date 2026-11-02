/** @file platform_probe_stm32.c
 * @brief 将框架短时耗时接口映射到现有 STM32 DWT 计数器。
 */
#include "../dwt/platform_probe.h"
#include "bsp_dwt.h"
#include <stddef.h>

uint32_t PlatformProbeStart(void)
{
    return DWT_ProbeStart();
}

void PlatformProbeDone(volatile PlatformProbe_t *probe, uint32_t start)
{
    DWT_ProbeDone(probe, start);
}

float PlatformProbeDeltaSeconds(uint32_t *previous)
{
    return DWT_GetDeltaT(previous);
}

uint8_t PlatformProbeElapsedSeconds(uint32_t start, uint32_t end, float *seconds)
{
    const uint32_t frequency_mhz = dwt_cpu_freq_mhz; // 只读取一次平台已经配置的周期频率。
    if (seconds == NULL || frequency_mhz == 0u) return 0u; // 未就绪时不交付错误间隔。
    *seconds = (float)(uint32_t)(end - start) / ((float)frequency_mhz * 1000000.0f); // 沿用电机滤波的换算与回绕规则。
    return 1u;
}
