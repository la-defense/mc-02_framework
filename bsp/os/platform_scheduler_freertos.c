/** @file platform_scheduler_freertos.c
 * @brief 将调度器运行状态查询绑定到现有 CMSIS 接口。
 */
#include "platform_os.h"
#include "cmsis_os.h"

uint8_t PlatformSchedulerIsRunning(void)
{
    return osKernelRunning() != 0u; // 保持原有运行状态判断。
}
