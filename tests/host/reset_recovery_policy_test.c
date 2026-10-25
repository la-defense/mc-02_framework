/**
 * @file reset_recovery_policy_test.c
 * @brief 使用生产纯策略检查跨复位额度、稳定窗口和人工急停保留。
 */
#include "reset_recovery_policy.h"
#include "test_check.h"

#include <stdint.h>

static int TestFaultResetLimit(void)
{
    ResetRecoveryRecord_t record;
    ResetRecoveryPolicyInitialize(&record);
    for (uint32_t attempt = 1u; attempt <= 3u; ++attempt)
    {
        ResetRecoveryPolicyMarkFault(&record, RESET_RECOVERY_CAUSE_CORE_TASK);
        const ResetRecoveryResetCause_t software_reset = {0u, 0u, 0u};
        TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, software_reset, 1u) ==
                   RESET_RECOVERY_STATUS_READY);
        TEST_CHECK(record.consecutive_attempts == attempt);
    }

    ResetRecoveryPolicyMarkFault(&record, RESET_RECOVERY_CAUSE_CPU_EXCEPTION);
    const ResetRecoveryResetCause_t software_reset = {0u, 0u, 0u};
    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, software_reset, 1u) ==
               RESET_RECOVERY_STATUS_LOCKED);
    TEST_CHECK(record.consecutive_attempts == 4u);
    TEST_CHECK(ResetRecoveryPolicyObserveControl(&record, 100u, 1u, 1u) == 0u);
    return 0;
}

static int TestResetClassification(void)
{
    ResetRecoveryRecord_t record;
    ResetRecoveryPolicyInitialize(&record);
    const ResetRecoveryResetCause_t ordinary_reset = {0u, 0u, 0u};
    for (uint32_t index = 0u; index < 8u; ++index)
        TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, ordinary_reset, 1u) ==
                   RESET_RECOVERY_STATUS_READY);
    TEST_CHECK(record.consecutive_attempts == 0u);

    const ResetRecoveryResetCause_t watchdog_reset = {0u, 1u, 0u};
    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, watchdog_reset, 1u) ==
               RESET_RECOVERY_STATUS_READY);
    TEST_CHECK(record.consecutive_attempts == 1u);
    const ResetRecoveryResetCause_t brownout_reset = {0u, 0u, 1u};
    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, brownout_reset, 1u) ==
               RESET_RECOVERY_STATUS_READY);
    TEST_CHECK(record.consecutive_attempts == 2u);

    const ResetRecoveryResetCause_t power_on = {1u, 0u, 1u};
    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, power_on, 0u) ==
               RESET_RECOVERY_STATUS_READY);
    TEST_CHECK(record.consecutive_attempts == 0u);
    TEST_CHECK(record.flags == 0u);

    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, ordinary_reset, 0u) ==
               RESET_RECOVERY_STATUS_LOCKED);
    TEST_CHECK(ResetRecoveryPolicyIsLocked(&record) != 0u);
    return 0;
}

static int TestStableWindowAndManualEstop(void)
{
    ResetRecoveryRecord_t record;
    ResetRecoveryPolicyInitialize(&record);
    ResetRecoveryPolicyMarkFault(&record, RESET_RECOVERY_CAUSE_FATAL_HANDLER);
    const ResetRecoveryResetCause_t software_reset = {0u, 0u, 0u};
    TEST_CHECK(ResetRecoveryPolicyOnBoot(&record, software_reset, 1u) ==
               RESET_RECOVERY_STATUS_READY);
    ResetRecoveryPolicySetManualEstop(&record, 1u);

    const uint32_t start = UINT32_MAX - 10000u;
    TEST_CHECK(ResetRecoveryPolicyObserveControl(&record, start, 1u, 1u) == 0u);
    TEST_CHECK(ResetRecoveryPolicyHasManualEstop(&record) != 0u);
    ResetRecoveryPolicySetManualEstop(&record, 0u);
    TEST_CHECK(ResetRecoveryPolicyObserveControl(&record, start, 1u, 1u) != 0u);
    TEST_CHECK(ResetRecoveryPolicyObserveControl(&record,
                                                start + RESET_RECOVERY_STABLE_MS - 1u,
                                                1u, 1u) == 0u);
    TEST_CHECK(record.consecutive_attempts == 1u);
    TEST_CHECK(ResetRecoveryPolicyObserveControl(&record,
                                                start + RESET_RECOVERY_STABLE_MS,
                                                1u, 1u) != 0u);
    TEST_CHECK(record.consecutive_attempts == 0u);
    return 0;
}

int main(void)
{
    TEST_CHECK(TestFaultResetLimit() == 0);
    TEST_CHECK(TestResetClassification() == 0);
    TEST_CHECK(TestStableWindowAndManualEstop() == 0);
    return 0;
}
