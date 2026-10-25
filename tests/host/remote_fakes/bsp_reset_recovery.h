/*
 * @file bsp_reset_recovery.h
 * @brief 为遥控任务测试替换跨复位记录写入。
 * @note 测试只验证遥控逻辑，BSP持久记录由单独生产测试覆盖。
 */
#ifndef MC02_REMOTE_FAKE_BSP_RESET_RECOVERY_H
#define MC02_REMOTE_FAKE_BSP_RESET_RECOVERY_H

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
