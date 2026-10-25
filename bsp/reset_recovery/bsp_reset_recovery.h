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
 * @param[in] cause 有效范围为RESET_RECOVERY_CAUSE_CORE_TASK至
 *                  RESET_RECOVERY_CAUSE_BROWNOUT；其他值忽略。
 * @return 无；存储未初始化或已锁定时不改写记录。
 * @note 异常路径只复制固定记录并更新CRC；不使用HAL、RTOS、日志、分配器或Flash。
 */
void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause);

/**
 * @brief 保存或清除需要跨 MCU 复位保留的人工急停标记。
 * @param[in] latched 非零保存急停，0清除；仅由明确的人工急停请求和解除入口调用。
 * @note 恢复存储未初始化时不修改状态；断电后标记不保证保留。
 */
void BSP_ResetRecoverySetManualEstop(uint8_t latched);

/**
 * @brief 查询锁存失能和恢复额度。
 * @param[out] output 接收复制状态；空指针或不可用时返回0且不改写。
 * @return 1 表示读取成功，0 表示参数或存储状态不可用。
 */
uint8_t BSP_ResetRecoveryGetState(BSP_ResetRecoveryState_t *output);

/**
 * @brief 查询当前是否必须保持复位恢复锁定。
 * @return 1表示锁定或存储不可用，0表示记录可用且未锁定。
 */
uint8_t BSP_ResetRecoveryIsLockedOut(void);

/**
 * @brief 观察控制与核心任务健康状态，连续稳定30秒后清除故障额度。
 * @param[in] now_ms HAL毫秒时刻，允许0值和回绕。
 * @param[in] control_ready 非0表示 RobotSafety 状态为 READY，0表示未就绪。
 * @param[in] all_core_healthy 非0表示当前期限检查确认所有核心任务正常，
 *                             0表示至少一项未健康。
 * @return 1表示记录有变化；0表示状态未变、记录未初始化或恢复已锁定。
 */
uint8_t BSP_ResetRecoveryObserveControl(uint32_t now_ms,
                                        uint8_t control_ready,
                                        uint8_t all_core_healthy);

#endif
