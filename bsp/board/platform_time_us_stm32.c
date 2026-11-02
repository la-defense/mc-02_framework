/** @file platform_time_us_stm32.c
 * @brief 将框架微秒时间接口绑定到当前 H723 计时实现。
 */
#include "../os/platform_sync.h"
#include "bsp_dwt.h"

uint64_t PlatformTimeNowUs(void)
{
    return DWT_GetTimeline_us(); // 沿用现有 64 位累计时钟及其同步方式。
}
