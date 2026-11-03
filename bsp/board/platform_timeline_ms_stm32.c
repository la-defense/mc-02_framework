/** @file platform_timeline_ms_stm32.c
 * @brief 为原有浮点毫秒诊断时间提供板级适配。
 */
#include "platform_probe.h"
#include "bsp_dwt.h"

float PlatformProbeTimelineMs(void)
{
    return DWT_GetTimeline_ms(); // 保持原先的时基、浮点换算和回绕处理。
}
