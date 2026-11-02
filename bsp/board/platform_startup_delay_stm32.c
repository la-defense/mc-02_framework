/** @file platform_startup_delay_stm32.c
 * @brief 将调度器启动前等待绑定到现有 DWT 忙等。
 */
#include "../os/platform_os.h"
#include "bsp_dwt.h"

void PlatformDelayBeforeSchedulerMs(uint32_t milliseconds)
{
    DWT_Delay((float)milliseconds / 1000.0f); // 保留原先的毫秒到秒换算。
}
