/*
 * @file bsp_reset_recovery.h
 * @brief 为主机替身中的任务监控和致命入口提供无硬件恢复接口。
 * @note 需要检查恢复记录时应链接生产 BSP；此头文件仅用于邻接模块测试。
 */
#ifndef MC02_HOST_BSP_RESET_RECOVERY_H
#define MC02_HOST_BSP_RESET_RECOVERY_H

#include "../../../bsp/reset_recovery/reset_recovery_policy.h"

static inline void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{
    (void)cause;
}

static inline void BSP_ResetRecoverySetManualEstop(uint8_t latched)
{
    (void)latched;
}

#endif
