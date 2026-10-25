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

/**
 * @brief 初始化空的有效恢复记录，不保留上一份状态。
 * @param[out] record 接收新记录；可空，空指针时不执行操作。
 * @return 无；成功时序号为1、额度和标志清零。
 */
void ResetRecoveryPolicyInitialize(ResetRecoveryRecord_t *record);

/**
 * @brief 根据复位来源更新额度；失败现场标记或看门狗／棕断电复位会消耗一次。
 * @param[in,out] record 记录输入输出；空指针时返回LOCKED且不写记录。
 *                      无效记录仅在cause.power_on非零时重建。
 * @param[in] cause 由BSP转换的复位来源，各成员使用0表示无、非0表示有；
 *                  POR优先建立新周期。
 * @param[in] record_valid 非零表示CRC、版本和固定长度检查均通过，0表示没有可信记录。
 * @return READY表示允许继续完整启动检查；LOCKED表示必须保持失能。
 *         非POR且记录无效时返回LOCKED。
 */
ResetRecoveryStatus_e ResetRecoveryPolicyOnBoot(ResetRecoveryRecord_t *record,
                                                ResetRecoveryResetCause_t cause,
                                                uint8_t record_valid);

/**
 * @brief 在复位前记录故障类别，供下一次启动累计。
 * @param[in,out] record 有效恢复记录；空指针时不执行操作。
 * @param[in] cause 有效范围为RESET_RECOVERY_CAUSE_CORE_TASK至
 *                  RESET_RECOVERY_CAUSE_BROWNOUT；其他值忽略。
 * @return 无；有效原因会设置待处理故障并更新序号，函数不访问持久介质。
 */
void ResetRecoveryPolicyMarkFault(ResetRecoveryRecord_t *record,
                                  ResetRecoveryCause_e cause);

/**
 * @brief 更新需要跨 MCU 复位保留的人工急停标记。
 * @param[in,out] record 有效恢复记录；空指针时不执行操作。
 * @param[in] latched 非零设置标记，0清除标记。
 * @return 无；成功时更新记录序号。
 */
void ResetRecoveryPolicySetManualEstop(ResetRecoveryRecord_t *record, uint8_t latched);

/**
 * @brief 只有 READY、核心任务全健康且无人工急停持续30秒才清除额度。
 * @param[in,out] record 当前有效记录。
 * @param[in] now_ms 单调毫秒计数，允许从 UINT32_MAX 回绕到0。
 * @param[in] control_ready 非零表示 RobotSafety 已进入 READY。
 * @param[in] all_core_healthy 非零表示当前期限检查确认所有核心任务均正常，
 *                             0表示不健康。
 * @return 1表示记录有变化，0表示未变化、记录为空或策略已锁定。
 */
uint8_t ResetRecoveryPolicyObserveControl(ResetRecoveryRecord_t *record,
                                          uint32_t now_ms,
                                          uint8_t control_ready,
                                          uint8_t all_core_healthy);

/**
 * @brief 查询当前记录是否处于跨复位锁定状态。
 * @param[in] record 恢复记录；可空。
 * @return 1表示记录含锁定标志；空指针或未锁定时返回0。
 */
uint8_t ResetRecoveryPolicyIsLocked(const ResetRecoveryRecord_t *record);

/**
 * @brief 查询人工急停是否需要在启动后重新锁存。
 * @param[in] record 恢复记录；可空。
 * @return 1表示设置了人工急停标志；空指针或未设置时返回0。
 */
uint8_t ResetRecoveryPolicyHasManualEstop(const ResetRecoveryRecord_t *record);

#endif
