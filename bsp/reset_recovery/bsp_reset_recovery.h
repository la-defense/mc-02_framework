/**
 * @file bsp_reset_recovery.h
 * @brief 保存故障复位额度和人工急停状态，并在启动及运行期执行固定恢复规则。
 * @note 必须在 main() 最早关断输出后、读取并清 RCC 复位标志前初始化。
 */
#ifndef BSP_RESET_RECOVERY_H
#define BSP_RESET_RECOVERY_H

#include <stdint.h>

#include "reset_recovery_policy.h"

typedef struct
{
    uint32_t consecutive_attempts;
    ResetRecoveryCause_e last_cause;
    uint8_t locked_out;
    uint8_t manual_estop_latched;
} BSP_ResetRecoveryState_t;

/**
 * @brief 根据 RCC_RSR 初始化复位恢复记录。
 * @param[in] reset_flags main() 读取的 RCC->RSR 原值；必须早于 RMVF 清标志。
 * @return 1 表示记录可用；0 表示运行配置无法安全保存，调用方必须保持失能。
 * @note 仅冷上电清额度；系统复位沿用 NOLOAD RAM 记录。人工急停不跨断电保留。
 */
uint8_t BSP_ResetRecoveryInit(uint32_t reset_flags);

/**
 * @brief 在关断输出或等待看门狗前标记一个故障复位原因。
 * @param[in] cause RESET_RECOVERY_CAUSE_*；非法原因忽略。
 * @note 异常路径只复制固定记录并更新CRC；不使用HAL、RTOS、日志、分配器或Flash。
 */
void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause);

/** @brief 保存／清除人工急停标记；仅明确的人工急停请求和解除入口调用。 */
void BSP_ResetRecoverySetManualEstop(uint8_t latched);

/**
 * @brief 查询锁存失能和恢复额度。
 * @param[out] output 接收复制状态；空指针或不可用时返回0且不改写。
 * @return 1 表示读取成功，0 表示参数或存储状态不可用。
 */
uint8_t BSP_ResetRecoveryGetState(BSP_ResetRecoveryState_t *output);

/** @brief 查询是否必须保持复位恢复锁定；存储不可用也按锁定处理。 */
uint8_t BSP_ResetRecoveryIsLockedOut(void);

/**
 * @brief 观察控制与核心任务健康状态，连续稳定30秒后清除故障额度。
 * @param[in] now_ms HAL毫秒时刻，允许0值和回绕。
 * @param[in] control_ready 非0表示 RobotSafety 状态为 READY。
 * @param[in] all_core_healthy 非0表示所有核心任务仍正常。
 * @return 1表示状态记录成功更新；0表示没有变化或记录不可用。
 */
uint8_t BSP_ResetRecoveryObserveControl(uint32_t now_ms,
                                        uint8_t control_ready,
                                        uint8_t all_core_healthy);

#endif
