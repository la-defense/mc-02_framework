/** @file platform_delay_500us_stm32.c
 * @brief 保留 BMI088 标定采样间原有的半毫秒忙等。
 */
#include "../dwt/platform_probe.h"
#include "bsp_dwt.h"

void PlatformProbeDelay500Us(void)
{
    DWT_Delay(0.0005); // 沿用原来的半毫秒等待及浮点换算。
}
