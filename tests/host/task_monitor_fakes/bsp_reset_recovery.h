/** @file bsp_reset_recovery.h
 * @brief 将任务故障记录交给主机测试观察。
 */
#ifndef MC02_TASK_MONITOR_FAKE_RESET_RECOVERY_H
#define MC02_TASK_MONITOR_FAKE_RESET_RECOVERY_H
#include "../../../bsp/reset_recovery/reset_recovery_policy.h"
void MC02TestRecordFault(ResetRecoveryCause_e cause);
static inline void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{
    MC02TestRecordFault(cause); // 在平台适配处保留故障类别和调用顺序。
}
#endif
