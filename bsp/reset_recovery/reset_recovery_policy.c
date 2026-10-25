/**
 * @file reset_recovery_policy.c
 * @brief 实现有限复位恢复、稳定清额度和人工急停保留规则。
 */
#include "reset_recovery_policy.h"

#include <stddef.h>

_Static_assert(sizeof(ResetRecoveryRecord_t) == 8u * sizeof(uint32_t),
               "恢复记录格式必须由八个32位字段组成");

static void ResetRecoveryPolicyPrepareRecord(ResetRecoveryRecord_t *record)
{
    if (record == NULL)
        return;
    record->header.fields.magic = RESET_RECOVERY_MAGIC;
    record->header.fields.version = RESET_RECOVERY_VERSION;
    record->sequence++;
    record->tail.fields.crc32 = 0u;
}

void ResetRecoveryPolicyInitialize(ResetRecoveryRecord_t *record)
{
    if (record == NULL)
        return;
    *record = (ResetRecoveryRecord_t){
        .header = {.fields = {
            .magic = RESET_RECOVERY_MAGIC,
            .version = RESET_RECOVERY_VERSION,
        }},
        .sequence = 1u,
        .consecutive_attempts = 0u,
        .flags = 0u,
        .last_cause = RESET_RECOVERY_CAUSE_NONE,
        .tail = {.fields = {.stable_since_ms = 0u, .crc32 = 0u}},
    };
}

ResetRecoveryStatus_e ResetRecoveryPolicyOnBoot(ResetRecoveryRecord_t *record,
                                                ResetRecoveryResetCause_t cause,
                                                uint8_t record_valid)
{
    if (record == NULL)
        return RESET_RECOVERY_STATUS_LOCKED;

    if (cause.power_on != 0u)
    {
        // 冷上电会丢失本记录；人工急停按已确认的B5规则不跨断电保留。
        ResetRecoveryPolicyInitialize(record);
        return RESET_RECOVERY_STATUS_READY;
    }

    if (record_valid == 0u || record->header.fields.magic != RESET_RECOVERY_MAGIC ||
        record->header.fields.version != RESET_RECOVERY_VERSION)
    {
        // 供电期间记录损坏时不猜测历史额度，保持输出锁定直到冷上电。
        ResetRecoveryPolicyInitialize(record);
        record->flags |= RESET_RECOVERY_FLAG_LOCKOUT;
        return RESET_RECOVERY_STATUS_LOCKED;
    }

    record->flags &= ~RESET_RECOVERY_FLAG_STABLE_ACTIVE;
    record->tail.fields.stable_since_ms = 0u;
    if ((record->flags & RESET_RECOVERY_FLAG_FAULT_PENDING) != 0u ||
        cause.watchdog != 0u || cause.brownout != 0u)
    {
        if (record->consecutive_attempts < UINT32_MAX)
            record->consecutive_attempts++;
        record->flags &= ~RESET_RECOVERY_FLAG_FAULT_PENDING;
        if (record->consecutive_attempts > RESET_RECOVERY_MAX_ATTEMPTS)
            record->flags |= RESET_RECOVERY_FLAG_LOCKOUT;
        if (cause.watchdog != 0u)
            record->last_cause = RESET_RECOVERY_CAUSE_WATCHDOG;
        else if (cause.brownout != 0u)
            record->last_cause = RESET_RECOVERY_CAUSE_BROWNOUT;
    }

    ResetRecoveryPolicyPrepareRecord(record);
    return ResetRecoveryPolicyIsLocked(record) ? RESET_RECOVERY_STATUS_LOCKED
                                                : RESET_RECOVERY_STATUS_READY;
}

void ResetRecoveryPolicyMarkFault(ResetRecoveryRecord_t *record,
                                  ResetRecoveryCause_e cause)
{
    if (record == NULL || cause <= RESET_RECOVERY_CAUSE_NONE ||
        cause > RESET_RECOVERY_CAUSE_BROWNOUT)
        return;
    record->flags |= RESET_RECOVERY_FLAG_FAULT_PENDING;
    record->flags &= ~RESET_RECOVERY_FLAG_STABLE_ACTIVE;
    record->tail.fields.stable_since_ms = 0u;
    record->last_cause = (uint32_t)cause;
    ResetRecoveryPolicyPrepareRecord(record);
}

void ResetRecoveryPolicySetManualEstop(ResetRecoveryRecord_t *record, uint8_t latched)
{
    if (record == NULL)
        return;
    if (latched != 0u)
        record->flags |= RESET_RECOVERY_FLAG_MANUAL_ESTOP;
    else
        record->flags &= ~RESET_RECOVERY_FLAG_MANUAL_ESTOP;
    ResetRecoveryPolicyPrepareRecord(record);
}

uint8_t ResetRecoveryPolicyObserveControl(ResetRecoveryRecord_t *record,
                                          uint32_t now_ms,
                                          uint8_t control_ready,
                                          uint8_t all_core_healthy)
{
    if (record == NULL || ResetRecoveryPolicyIsLocked(record))
        return 0u;

    const uint32_t disqualifying_flags = RESET_RECOVERY_FLAG_MANUAL_ESTOP |
                                         RESET_RECOVERY_FLAG_FAULT_PENDING;
    if (control_ready == 0u || all_core_healthy == 0u ||
        (record->flags & disqualifying_flags) != 0u)
    {
        if ((record->flags & RESET_RECOVERY_FLAG_STABLE_ACTIVE) == 0u)
            return 0u;
        record->flags &= ~RESET_RECOVERY_FLAG_STABLE_ACTIVE;
        record->tail.fields.stable_since_ms = 0u;
        ResetRecoveryPolicyPrepareRecord(record);
        return 1u;
    }

    if ((record->flags & RESET_RECOVERY_FLAG_STABLE_ACTIVE) == 0u)
    {
        record->flags |= RESET_RECOVERY_FLAG_STABLE_ACTIVE;
        record->tail.fields.stable_since_ms = now_ms;
        ResetRecoveryPolicyPrepareRecord(record);
        return 1u;
    }

    if ((uint32_t)(now_ms - record->tail.fields.stable_since_ms) < RESET_RECOVERY_STABLE_MS ||
        record->consecutive_attempts == 0u)
        return 0u;

    record->consecutive_attempts = 0u;
    record->flags &= ~RESET_RECOVERY_FLAG_STABLE_ACTIVE;
    record->tail.fields.stable_since_ms = 0u;
    record->last_cause = RESET_RECOVERY_CAUSE_NONE;
    ResetRecoveryPolicyPrepareRecord(record);
    return 1u;
}

uint8_t ResetRecoveryPolicyIsLocked(const ResetRecoveryRecord_t *record)
{
    return (record != NULL && (record->flags & RESET_RECOVERY_FLAG_LOCKOUT) != 0u)
               ? 1u
               : 0u;
}

uint8_t ResetRecoveryPolicyHasManualEstop(const ResetRecoveryRecord_t *record)
{
    return (record != NULL && (record->flags & RESET_RECOVERY_FLAG_MANUAL_ESTOP) != 0u)
               ? 1u
               : 0u;
}
