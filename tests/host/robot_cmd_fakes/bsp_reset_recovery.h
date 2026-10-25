/*
 * @file bsp_reset_recovery.h
 * @brief 为机器人命令测试替换跨复位状态写入。
 * @note 恢复存储由单独的生产 BSP 测试验证。
 */
#ifndef MC02_ROBOT_CMD_FAKE_BSP_RESET_RECOVERY_H
#define MC02_ROBOT_CMD_FAKE_BSP_RESET_RECOVERY_H

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
