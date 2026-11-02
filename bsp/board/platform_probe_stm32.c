/** @file platform_probe_stm32.c
 * @brief 将框架短时耗时接口映射到现有 STM32 DWT 计数器。
 */
#include "../dwt/platform_probe.h"
#include "bsp_dwt.h"

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
