/*
 * @file bsp_reset_recovery.h
 * @brief 为遥控任务测试替换跨复位记录写入。
 * @note 测试只验证遥控逻辑，BSP持久记录由单独生产测试覆盖。
 */
#ifndef MC02_REMOTE_FAKE_BSP_RESET_RECOVERY_H
#define MC02_REMOTE_FAKE_BSP_RESET_RECOVERY_H

#include "../../../bsp/reset_recovery/reset_recovery_policy.h"

#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
extern uint8_t remote_test_reset_estop_latched; // 由遥控联测读取安全模块最后一次提交的标志。
extern uint32_t remote_test_reset_estop_writes; // 检查重复帧是否反复写入跨复位记录。
#endif

static inline void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{
    (void)cause;
}

static inline void BSP_ResetRecoverySetManualEstop(uint8_t latched)
{
#if defined(MC02_REMOTE_CAPTURE_RESET_ESTOP)
    remote_test_reset_estop_latched = latched;
    ++remote_test_reset_estop_writes;
#else
    (void)latched;
#endif
}

#endif
