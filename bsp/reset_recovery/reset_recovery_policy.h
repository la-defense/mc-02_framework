/**
 * @file reset_recovery_policy.h
 * @brief 纯计算跨复位恢复额度与手动急停记录的状态变化。
 * @note 该模块不读硬件、不分配内存；固定记录由 BSP 层放入 NOLOAD RAM。
 */
#ifndef RESET_RECOVERY_POLICY_H
#define RESET_RECOVERY_POLICY_H

#include <stdint.h>

#define RESET_RECOVERY_MAGIC 0x52564359u
#define RESET_RECOVERY_VERSION 1u
#define RESET_RECOVERY_MAX_ATTEMPTS 3u
#define RESET_RECOVERY_STABLE_MS 30000u

typedef enum
{
    RESET_RECOVERY_CAUSE_NONE = 0,
    RESET_RECOVERY_CAUSE_CORE_TASK = 1,
    RESET_RECOVERY_CAUSE_CPU_EXCEPTION = 2,
    RESET_RECOVERY_CAUSE_ASSERT = 3,
    RESET_RECOVERY_CAUSE_STACK_OVERFLOW = 4,
    RESET_RECOVERY_CAUSE_FATAL_HANDLER = 5,
    RESET_RECOVERY_CAUSE_WATCHDOG = 6,
    RESET_RECOVERY_CAUSE_BROWNOUT = 7
} ResetRecoveryCause_e;

enum
{
    RESET_RECOVERY_FLAG_FAULT_PENDING = 1u << 0,
    RESET_RECOVERY_FLAG_LOCKOUT = 1u << 1,
    RESET_RECOVERY_FLAG_MANUAL_ESTOP = 1u << 2,
    RESET_RECOVERY_FLAG_STABLE_ACTIVE = 1u << 3
};

/** @brief 跨系统复位保存的固定八字记录；持久化层另行检查CRC。 */
typedef union
{
    struct
    {
        uint32_t magic;
        uint32_t version;
    } fields;
    uint64_t ecc_flush_word;
} ResetRecoveryHeader_t;

typedef union
{
    struct
    {
        uint32_t stable_since_ms;
        uint32_t crc32;
    } fields;
    uint64_t ecc_flush_word;
} ResetRecoveryTail_t;

typedef struct
{
    ResetRecoveryHeader_t header;
    uint32_t sequence;
    uint32_t consecutive_attempts;
    uint32_t flags;
    uint32_t last_cause;
    ResetRecoveryTail_t tail;
} ResetRecoveryRecord_t;

typedef enum
{
    RESET_RECOVERY_STATUS_READY = 0,
    RESET_RECOVERY_STATUS_LOCKED
} ResetRecoveryStatus_e;

/** @brief 由 RCC 复位标志抽象出的来源；POR 与其他标志可能同时出现。 */
typedef struct
{
    uint8_t power_on;
    uint8_t watchdog;
    uint8_t brownout;
} ResetRecoveryResetCause_t;

/** @brief 初始化空记录，不保留上一份状态。 */
void ResetRecoveryPolicyInitialize(ResetRecoveryRecord_t *record);

/**
 * @brief 根据复位来源更新额度；失败现场标记或看门狗／棕断电复位会消耗一次。
 * @param[in,out] record 已校验记录；无效记录只在 power_on 为真时可安全重建。
 * @param[in] cause 由 BSP 从 RCC 复位标志转换的来源。
 * @param[in] record_valid 非零表示 CRC、版本和长度均通过。
 * @return READY 表示允许进行完整启动检查；LOCKED 表示启动后仍须保持失能。
 */
ResetRecoveryStatus_e ResetRecoveryPolicyOnBoot(ResetRecoveryRecord_t *record,
                                                ResetRecoveryResetCause_t cause,
                                                uint8_t record_valid);

/** @brief 在复位前记录故障类别，供下一次启动累计；不会访问持久介质。 */
void ResetRecoveryPolicyMarkFault(ResetRecoveryRecord_t *record,
                                  ResetRecoveryCause_e cause);

/** @brief 更新需要跨 MCU 复位保留的人工急停标记。 */
void ResetRecoveryPolicySetManualEstop(ResetRecoveryRecord_t *record, uint8_t latched);

/**
 * @brief 只有 READY、核心任务全健康且无人工急停持续30秒才清除额度。
 * @param[in,out] record 当前有效记录。
 * @param[in] now_ms 单调毫秒计数，允许从 UINT32_MAX 回绕到0。
 * @param[in] control_ready 非零表示 RobotSafety 已进入 READY。
 * @param[in] all_core_healthy 非零表示所有核心任务均正常。
 * @return 1 表示记录有变化，0 表示未变化。
 */
uint8_t ResetRecoveryPolicyObserveControl(ResetRecoveryRecord_t *record,
                                          uint32_t now_ms,
                                          uint8_t control_ready,
                                          uint8_t all_core_healthy);

/** @brief 查询当前记录是否处于跨复位锁定状态。 */
uint8_t ResetRecoveryPolicyIsLocked(const ResetRecoveryRecord_t *record);

/** @brief 查询人工急停是否需要在启动后重新锁存。 */
uint8_t ResetRecoveryPolicyHasManualEstop(const ResetRecoveryRecord_t *record);

#endif
