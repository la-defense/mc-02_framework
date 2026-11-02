/** @file task_monitor_delay_fake.c
 * @brief 将主机测试的任务等待映射到可控 CMSIS 时间替身。
 */
#include "../../bsp/os/platform_os.h"
#include "cmsis_os.h"

void PlatformTaskDelayMs(uint32_t milliseconds)
{
    osDelay(milliseconds); // 模拟生产平台后端沿用的毫秒等待。
}
