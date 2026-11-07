/** @file clock_startup_test.c
 * @brief 运行生产 H723 启动适配器，模拟停表、VOS 失败和有限复位。
 */
#include "platform_clock_startup.h"
#include "bsp_reset_recovery.h"
#include "init_diagnostic.h"
#include "main.h"
#include "test_check.h"
#include <setjmp.h>
#include <stdint.h>
#include <string.h>

ClockTestDebug_t clock_test_debug;
ClockTestDwt_t clock_test_dwt;
ClockTestWatchdog_t clock_test_watchdog;
volatile uint32_t uwTick;
uint32_t clock_test_irq;
uint32_t clock_test_ipsr;
uint8_t clock_test_vos;
static uint8_t locked, outputs_off, reset_requested, flags_cleared;
static uint8_t advance_raw_tick;
static ResetRecoveryCause_e saved_cause;
static InitDiagnosticStage_e saved_stage;
static InitDiagnosticReason_e saved_reason;
static uint8_t saved_module;
static uint16_t saved_resource;
static jmp_buf failure_exit;

uint32_t HAL_GetTick(void);
uint32_t ClockTestReadTick(void) { if (advance_raw_tick) ++uwTick; return uwTick; }
void BSP_SafetyStartupLatchOutputsOff(void) { outputs_off = 1u; }
void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{ TEST_CHECK(outputs_off && clock_test_irq); saved_cause = cause; }
uint8_t BSP_ResetRecoveryIsLockedOut(void) { return locked; }
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK(outputs_off && clock_test_irq); // 故障记录之前必须已经关断。
    saved_stage = stage; saved_module = module; saved_reason = reason; saved_resource = resource;
}
void ClockTestClearResetFlags(void) { TEST_CHECK(saved_cause != RESET_RECOVERY_CAUSE_NONE); flags_cleared = 1u; }
void NVIC_SystemReset(void) { TEST_CHECK(flags_cleared); reset_requested = 1u; longjmp(failure_exit, 1); }
void ClockTestNop(void) { longjmp(failure_exit, 1); } // 模拟已锁定时留在关断循环。

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个场景单独运行，避免启动状态互相影响。
    uwTick = 23u; // 模拟尚未改变的真实 HAL 毫秒计数。
    if (strcmp(argv[1], "hal-before-begin") != 0)
    {
        PlatformClockStartupBegin(); // 启动固定期限并启用周期计数。
        TEST_CHECK(clock_test_debug.DEMCR && clock_test_dwt.CTRL); // 本板适配器启用计数来源。
    }
    if (strncmp(argv[1], "peripheral-", 11u) == 0)
    {
        TEST_CHECK(!PlatformPeripheralStartupBegin()); // 主时钟保护尚未结束时不能重叠。
        advance_raw_tick = 1u; // 模拟主时钟切换后真实时基正常。
        TEST_CHECK(PlatformClockStartupFinish());
        advance_raw_tick = 0u;
        TEST_CHECK(PlatformPeripheralStartupBegin()); // 记录外设阶段的固定起点。
        TEST_CHECK(!PlatformPeripheralStartupBegin()); // 不能重置已经开始的总期限。
        TEST_CHECK(PlatformPeripheralStartupEnter());
        TEST_CHECK(!PlatformPeripheralStartupEnter()); // 不允许嵌套使用保护。
        if (!strcmp(argv[1], "peripheral-stopped"))
        {
            const uint32_t first = HAL_GetTick(); // 周期和真实毫秒均保持不动。
            for (unsigned i = 0u; i < 6u; ++i) (void)HAL_GetTick(); // 耗尽总查询次数。
            TEST_CHECK((uint32_t)(HAL_GetTick() - first) > HAL_UART_TIMEOUT_VALUE); // HAL 的大超时也能退出。
            TEST_CHECK(PlatformClockRawTick() == uwTick); // 日志读取原始时间，不记录临时偏移。
            TEST_CHECK(!PlatformPeripheralStartupLeave()); // 临时的 HAL 时间不能变成成功。
            TEST_CHECK(HAL_GetTick() == uwTick); // 离开后恢复原始时钟。
            TEST_CHECK(!PlatformPeripheralStartupFinish()); // 总期限已用尽。
            return 0;
        }
        if (!strcmp(argv[1], "peripheral-cycle"))
        {
            clock_test_dwt.CYCCNT = 40u; // HAL 没查询时间也应在退出时检查期限。
            TEST_CHECK(!PlatformPeripheralStartupLeave());
            TEST_CHECK(!PlatformPeripheralStartupFinish());
            return 0;
        }
        if (!strcmp(argv[1], "peripheral-isr"))
        {
            clock_test_dwt.CYCCNT = 40u; // 模拟外设操作已经超期。
            (void)HAL_GetTick(); // 启动线程先触发临时时间。
            clock_test_ipsr = 1u; // 模拟中断打断启动线程。
            TEST_CHECK(HAL_GetTick() == uwTick); // 中断和其数据时间不能看到临时偏移。
            TEST_CHECK(!PlatformPeripheralStartupLeave()); // 中断不能结束线程的保护。
            clock_test_ipsr = 0u;
            TEST_CHECK(!PlatformPeripheralStartupLeave());
            TEST_CHECK(!PlatformPeripheralStartupFinish());
            return 0;
        }
        TEST_CHECK(PlatformPeripheralStartupLeave()); // 正常操作后回到真实时钟。
        TEST_CHECK(HAL_GetTick() == uwTick);
        if (!strcmp(argv[1], "peripheral-sequence"))
        {
            clock_test_dwt.CYCCNT = 40u; // 不因下一次操作而刷新总期限。
            TEST_CHECK(!PlatformPeripheralStartupEnter());
            TEST_CHECK(!PlatformPeripheralStartupFinish());
            return 0;
        }
        TEST_CHECK(!strcmp(argv[1], "peripheral-success"));
        TEST_CHECK(PlatformPeripheralStartupFinish());
        TEST_CHECK(!PlatformPeripheralStartupFinish()); // 已结束的期限不能再次使用。
        TEST_CHECK(!PlatformPeripheralStartupBegin()); // 结束后也不能刷新本次上电的总期限。
        return 0;
    }
    if (!strcmp(argv[1], "success"))
    {
        clock_test_vos = 1u; // 模拟电压立即就绪。
        TEST_CHECK(PlatformClockStartupWaitVos()); // 就绪时不等待多余次数。
        TEST_CHECK(HAL_GetTick() == 23u && !clock_test_irq); // 正常路径保留真实时间及中断状态。
        clock_test_irq = 1u; // 模拟进入前已经屏蔽中断的调用方。
        TEST_CHECK(HAL_GetTick() == 23u && clock_test_irq == 1u); // 查询后保持外层屏蔽状态。
        clock_test_irq = 0u; // 恢复中断，以便测试时基前进。
        advance_raw_tick = 1u; // 模拟切换后的 HAL 毫秒中断仍在运行。
        TEST_CHECK(PlatformClockStartupFinish()); // 原始时基前进且总期限未耗尽。
        advance_raw_tick = 0u; // 保留最后一次真实时钟值供比较。
        TEST_CHECK(HAL_GetTick() == uwTick); // 退出保护后恢复原始 HAL 时间。
        return 0;
    }
    if (!strcmp(argv[1], "vos-timeout"))
    {
        TEST_CHECK(!PlatformClockStartupWaitVos()); // 周期停止且 VOS 未就绪时按次数退出。
        return 0;
    }
    if (!strcmp(argv[1], "tick-stopped"))
    {
        const uint32_t initial = HAL_GetTick(); // 捕获 HAL 等待起点。
        for (unsigned i = 0; i < 4u; ++i) (void)HAL_GetTick(); // 原始时钟和周期计数均保持不变。
        TEST_CHECK((uint32_t)(HAL_GetTick() - initial) > 5000u); // HAL 的固定超时循环可以退出。
        TEST_CHECK(!PlatformClockStartupFinish()); // 临时偏移不能被当成成功。
        TEST_CHECK(HAL_GetTick() == uwTick && uwTick == 23u); // 未改写真实 HAL 时钟。
        return 0;
    }
    if (!strcmp(argv[1], "cycle-timeout"))
    {
        clock_test_dwt.CYCCNT = 50u; // 达到本测试的总周期上限。
        TEST_CHECK((uint32_t)(HAL_GetTick() - 23u) > 5000u); // 周期期限也触发 HAL 超时。
        TEST_CHECK(!PlatformClockStartupFinish()); // 外层必须拒绝该次配置。
        return 0;
    }
    if (!strcmp(argv[1], "raw-stopped"))
    {
        TEST_CHECK(!PlatformClockStartupFinish()); // 即使 HAL 步骤均立即返回，停滞的时基也不能通过。
        TEST_CHECK(HAL_GetTick() == 23u); // 失败后仍保留真实计数，不发布伪造时间。
        return 0;
    }
    if (!strcmp(argv[1], "finish-total-timeout"))
    {
        clock_test_dwt.CYCCNT = 50u; // 总期限已到，不能因 tick 刚好前进而接受成功。
        advance_raw_tick = 1u; // 模拟原始时基已经恢复。
        TEST_CHECK(!PlatformClockStartupFinish()); // 检查时不能重新开始总期限。
        return 0;
    }
    PlatformClockFailure_t requested_failure = PLATFORM_CLOCK_FAILURE_OSCILLATOR;
    PlatformClockFailure_t expected_failure = requested_failure;
    ResetRecoveryCause_e expected_cause = RESET_RECOVERY_CAUSE_CLOCK_OSCILLATOR;
    if (!strcmp(argv[1], "locked")) locked = 1u; // 模拟恢复次数已用尽。
    else if (!strcmp(argv[1], "supply-failure"))
    { requested_failure = expected_failure = PLATFORM_CLOCK_FAILURE_SUPPLY; expected_cause = RESET_RECOVERY_CAUSE_CLOCK_SUPPLY; }
    else if (!strcmp(argv[1], "vos-failure"))
    { requested_failure = expected_failure = PLATFORM_CLOCK_FAILURE_VOS; expected_cause = RESET_RECOVERY_CAUSE_CLOCK_VOS; }
    else if (!strcmp(argv[1], "switch-failure"))
    { requested_failure = expected_failure = PLATFORM_CLOCK_FAILURE_SWITCH; expected_cause = RESET_RECOVERY_CAUSE_CLOCK_SWITCH; }
    else if (!strcmp(argv[1], "hal-failure") || !strcmp(argv[1], "hal-before-begin"))
    { requested_failure = expected_failure = PLATFORM_CLOCK_FAILURE_HAL; expected_cause = RESET_RECOVERY_CAUSE_CLOCK_HAL; }
    else if (!strcmp(argv[1], "periph-failure"))
    { requested_failure = expected_failure = PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE; expected_cause = RESET_RECOVERY_CAUSE_PERIPHERAL_DEADLINE; }
    else if (!strcmp(argv[1], "invalid-failure"))
    { requested_failure = (PlatformClockFailure_t)0; expected_failure = PLATFORM_CLOCK_FAILURE_DEADLINE; expected_cause = RESET_RECOVERY_CAUSE_CLOCK_DEADLINE; }
    else if (!strcmp(argv[1], "expired-reset"))
    {
        clock_test_dwt.CYCCNT = 50u; // 先触发时钟配置的固定总期限。
        (void)HAL_GetTick(); // 让生产保护记录已经到期。
        expected_failure = PLATFORM_CLOCK_FAILURE_DEADLINE; // 实际超期比 HAL 的振荡器错误更具体。
        expected_cause = RESET_RECOVERY_CAUSE_CLOCK_DEADLINE;
    }
    else TEST_CHECK(!strcmp(argv[1], "reset"));
    if (setjmp(failure_exit) == 0)
        PlatformClockStartupFail(requested_failure); // 失败入口不得正常返回。
    TEST_CHECK(outputs_off && clock_test_irq); // 返回测试后确认仍保持关断和中断屏蔽。
    TEST_CHECK(saved_stage == (expected_failure == PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE ? INIT_STAGE_BSP : INIT_STAGE_CLOCK) && saved_module == INIT_MODULE_CLOCK &&
               saved_reason == INIT_FAILURE_CLOCK && saved_resource == expected_failure);
    TEST_CHECK(saved_cause == expected_cause); // 复位记录保留具体阶段。
    TEST_CHECK(reset_requested == !locked); // 仅有恢复额度时请求复位。
    TEST_CHECK(flags_cleared == !locked); // 已锁定时不重启，也不清除现场标志。
    TEST_CHECK(!locked || clock_test_watchdog.KR == 0xAAAAu); // 已锁定时重载可能仍在运行的硬件看门狗。
    return 0;
}
