/*
 * @file reset_recovery_test.c
 * @brief 通过生产BSP接口检查跨复位额度、锁定和稳定恢复行为。
 * @note 使用可控寄存器模型，不触发真实复位，也不覆盖硬件保留行为验收。
 */
#include "bsp_reset_recovery.h"
#include "main.h"
#include "test_check.h"

#include <stdint.h>

MC02_ResetRecoverySCB_t mc02_reset_recovery_scb;
uint32_t mc02_reset_recovery_primask;
uint32_t mc02_reset_recovery_dsb_count;

/* 只在这个主机测试目标中提供，模拟复位记录写到一半时掉电。 */
void BSP_ResetRecoveryTestTearInactiveSlot(void);

static int TestBootAndQuota(void)
{
    BSP_ResetRecoveryState_t state = {.consecutive_attempts = 99u};
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) == 0u);
    TEST_CHECK(state.consecutive_attempts == 99u);

    // 没有有效记录且不是冷上电时，生产接口必须以锁定启动。
    TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.locked_out != 0u);

    // H723冷上电组合包含POR和BOR标志；冷上电从新的RAM周期开始。
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.locked_out == 0u && state.consecutive_attempts == 0u);

    for (uint32_t attempt = 1u; attempt <= RESET_RECOVERY_MAX_ATTEMPTS; ++attempt)
    {
        BSP_ResetRecoveryMarkFault(RESET_RECOVERY_CAUSE_CORE_TASK);
        TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);
        TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
        TEST_CHECK(state.locked_out == 0u);
        TEST_CHECK(state.consecutive_attempts == attempt);
    }

    BSP_ResetRecoveryMarkFault(RESET_RECOVERY_CAUSE_CPU_EXCEPTION);
    TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.locked_out != 0u);
    TEST_CHECK(state.consecutive_attempts == RESET_RECOVERY_MAX_ATTEMPTS + 1u);

    // 普通复位不能清锁定；重新上电按已确认的冷启动规则重新计额度。
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PINRSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u && state.locked_out != 0u);
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.locked_out == 0u && state.consecutive_attempts == 0u);
    TEST_CHECK(mc02_reset_recovery_dsb_count > 0u);
    return 0;
}

static int TestResetCausesAndManualEstop(void)
{
    BSP_ResetRecoveryState_t state;
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF) != 0u);
    BSP_ResetRecoverySetManualEstop(1u);
    TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.manual_estop_latched != 0u);

    BSP_ResetRecoverySetManualEstop(0u);
    TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.manual_estop_latched == 0u);

    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_IWDG1RSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.consecutive_attempts == 1u);
    TEST_CHECK(state.last_cause == RESET_RECOVERY_CAUSE_WATCHDOG);

    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_BORRSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.consecutive_attempts == 2u);
    TEST_CHECK(state.last_cause == RESET_RECOVERY_CAUSE_BROWNOUT);

    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_WWDG1RSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.consecutive_attempts == 3u);
    TEST_CHECK(state.last_cause == RESET_RECOVERY_CAUSE_WATCHDOG);
    return 0;
}

static int TestStableWindowAndUnavailableStorage(void)
{
    BSP_ResetRecoveryState_t state;
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF) != 0u);
    BSP_ResetRecoveryMarkFault(RESET_RECOVERY_CAUSE_ASSERT);
    TEST_CHECK(BSP_ResetRecoveryInit(0u) != 0u);

    const uint32_t start = UINT32_MAX - 10000u;
    TEST_CHECK(BSP_ResetRecoveryObserveControl(start, 1u, 1u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryObserveControl(start + 12000u, 1u, 0u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryObserveControl(start + 12001u, 1u, 1u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryObserveControl(
                   start + 12001u + RESET_RECOVERY_STABLE_MS - 1u, 1u, 1u) == 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u && state.consecutive_attempts == 1u);
    TEST_CHECK(BSP_ResetRecoveryObserveControl(
                   start + 12001u + RESET_RECOVERY_STABLE_MS, 1u, 1u) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u && state.consecutive_attempts == 0u);

    mc02_reset_recovery_scb.CCR = SCB_CCR_DC_Msk;
    TEST_CHECK(BSP_ResetRecoveryInit(0u) == 0u);
    TEST_CHECK(BSP_ResetRecoveryIsLockedOut() != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) == 0u);
    mc02_reset_recovery_scb.CCR = 0u;
    return 0;
}

static int TestTornRecordFailsClosed(void)
{
    BSP_ResetRecoveryState_t state;
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF) != 0u);

    // 新周期只有一个有效槽；模拟另一个槽在提交故障记录时被复位打断。
    BSP_ResetRecoveryTestTearInactiveSlot();
    TEST_CHECK(BSP_ResetRecoveryInit(RCC_RSR_PINRSTF) != 0u);
    TEST_CHECK(BSP_ResetRecoveryGetState(&state) != 0u);
    TEST_CHECK(state.locked_out != 0u);
    return 0;
}

int main(void)
{
    TEST_CHECK(TestBootAndQuota() == 0);
    TEST_CHECK(TestResetCausesAndManualEstop() == 0);
    TEST_CHECK(TestStableWindowAndUnavailableStorage() == 0);
    TEST_CHECK(TestTornRecordFailsClosed() == 0);
    return 0;
}
