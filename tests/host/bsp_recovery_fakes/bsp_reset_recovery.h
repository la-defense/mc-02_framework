/*
 * @file bsp_reset_recovery.h
 * @brief 为不测试恢复存储的模块替换跨复位恢复动作。
 * @note 仅测试构建使用；生产行为由 bsp/reset_recovery 实现。
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
