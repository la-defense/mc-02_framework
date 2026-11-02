/** @file platform_safety_stm32.c
 * @brief 将任务监控的安全入口绑定到 MC-02 原有关断及恢复实现。
 */
#include "../os/platform_safety.h"
#include "bsp_reset_recovery.h"
#include "bsp_safety.h"
#include "bsp_watchdog.h"

void PlatformRecordCoreTaskFault(void)
{
    BSP_ResetRecoveryMarkFault(RESET_RECOVERY_CAUSE_CORE_TASK); // 复位后按核心任务故障统计恢复额度。
}

void PlatformSafetyLatchOutputsOff(void)
{
    BSP_SafetyLatchOutputsOff(); // 直接关断输出，返回后保持普通中断关闭。
}

void PlatformWatchdogFeed(void)
{
    BSP_WatchdogFeed(); // 沿用现有硬件看门狗重载规则。
}
