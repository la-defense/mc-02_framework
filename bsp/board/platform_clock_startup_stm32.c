/** @file platform_clock_startup_stm32.c
 * @brief 为 H723 HAL 主时钟配置提供固定期限与有限故障复位。
 */
#include "platform_clock_startup.h"
#include "platform_clock_guard.h"
#include "platform_startup_wait.h"
#include "bsp_reset_recovery.h"
#include "bsp_safety.h"
#include "init_diagnostic.h"
#include "main.h"
#include <limits.h>
#include <stddef.h>

/* 复位后 HSI 为 64 MHz；这些上限仍需在实际板上验证。 */
#ifndef CLOCK_STARTUP_TOTAL_CYCLES
#define CLOCK_STARTUP_TOTAL_CYCLES 320000000u /* 约 5 s 的 64 MHz 周期数。 */
#endif
#ifndef CLOCK_STARTUP_TOTAL_POLLS
#define CLOCK_STARTUP_TOTAL_POLLS 32000000u /* 计数器停止时的总查询上限。 */
#endif
#ifndef CLOCK_STARTUP_VOS_CYCLES
#define CLOCK_STARTUP_VOS_CYCLES 6400000u /* 约 100 ms 的 64 MHz 周期数。 */
#endif
#ifndef CLOCK_STARTUP_VOS_POLLS
#define CLOCK_STARTUP_VOS_POLLS 6400000u /* VOS 计数停止时的检查上限。 */
#endif
#ifndef CLOCK_STARTUP_TICK_CYCLES
#define CLOCK_STARTUP_TICK_CYCLES 48000000u /* 切换后约 100 ms 的 480 MHz 周期数。 */
#endif
#ifndef CLOCK_STARTUP_TICK_POLLS
#define CLOCK_STARTUP_TICK_POLLS 48000000u /* 时基和周期计数均停止时的检查上限。 */
#endif
#ifndef CLOCK_STARTUP_READ_RAW_TICK
#define CLOCK_STARTUP_READ_RAW_TICK() (uwTick) /* 从 HAL 的原始毫秒计数读取。 */
#endif
#ifndef PERIPHERAL_STARTUP_TOTAL_CYCLES
#define PERIPHERAL_STARTUP_TOTAL_CYCLES 1440000000u /* 480 MHz 下约 3 s；板上时间需验证。 */
#endif
#ifndef PERIPHERAL_STARTUP_TOTAL_POLLS
#define PERIPHERAL_STARTUP_TOTAL_POLLS 32000000u /* 周期计数停止时的总查询上限。 */
#endif
#define PERIPHERAL_EXPIRED_STEP_MS 0x02000001u /* 大于当前 HAL UART 初始化超时。 */

_Static_assert(CLOCK_GUARD_EXPIRED_STEP_MS > CLOCKSWITCH_TIMEOUT_VALUE,
               "clock guard step must exceed the HAL clock switch timeout");
/* HAL 的 PWR_FLAG_SETTING_DELAY 在驱动 .c 内定义为 1000 ms，升级驱动时须重新核对。 */
_Static_assert(CLOCK_GUARD_EXPIRED_STEP_MS > 1000u,
               "clock guard step must exceed the HAL supply timeout");
_Static_assert(CLOCK_GUARD_EXPIRED_STEP_MS > HSE_TIMEOUT_VALUE &&
               CLOCK_GUARD_EXPIRED_STEP_MS > HSI_TIMEOUT_VALUE &&
               CLOCK_GUARD_EXPIRED_STEP_MS > HSI48_TIMEOUT_VALUE &&
               CLOCK_GUARD_EXPIRED_STEP_MS > PLL_TIMEOUT_VALUE,
               "clock guard step must exceed oscillator timeouts");
_Static_assert(PERIPHERAL_EXPIRED_STEP_MS > HAL_UART_TIMEOUT_VALUE,
               "peripheral guard step must exceed the HAL UART timeout");
_Static_assert(PERIPHERAL_STARTUP_TOTAL_CYCLES > 0u && PERIPHERAL_STARTUP_TOTAL_CYCLES <= INT32_MAX &&
               PERIPHERAL_STARTUP_TOTAL_POLLS > 0u, "peripheral startup deadline must be valid");
_Static_assert((RESET_RECOVERY_CAUSE_CLOCK_HAL - RESET_RECOVERY_CAUSE_CLOCK_SUPPLY) ==
               (PLATFORM_CLOCK_FAILURE_HAL - PLATFORM_CLOCK_FAILURE_SUPPLY),
               "clock failure and recovery cause ranges must match");
_Static_assert((RESET_RECOVERY_CAUSE_PERIPHERAL_DEADLINE - RESET_RECOVERY_CAUSE_CLOCK_SUPPLY) ==
               (PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE - PLATFORM_CLOCK_FAILURE_SUPPLY),
               "peripheral deadline recovery cause must match");
_Static_assert((RESET_RECOVERY_CAUSE_STARTUP_WATCHDOG - RESET_RECOVERY_CAUSE_CLOCK_SUPPLY) ==
               (PLATFORM_CLOCK_FAILURE_WATCHDOG - PLATFORM_CLOCK_FAILURE_SUPPLY),
               "startup watchdog recovery cause must match");
_Static_assert(CLOCK_STARTUP_TOTAL_CYCLES > 0u && CLOCK_STARTUP_TOTAL_CYCLES <= INT32_MAX &&
               CLOCK_STARTUP_VOS_CYCLES > 0u && CLOCK_STARTUP_VOS_CYCLES <= INT32_MAX &&
               CLOCK_STARTUP_TICK_CYCLES > 0u && CLOCK_STARTUP_TICK_CYCLES <= INT32_MAX &&
               CLOCK_STARTUP_TOTAL_POLLS > 0u && CLOCK_STARTUP_VOS_POLLS > 0u &&
               CLOCK_STARTUP_TICK_POLLS > 0u,
               "clock startup deadlines must be valid for wrap-safe waits");

static PlatformClockGuard_t clock_guard; // 只有早期主时钟配置使用此状态。
static volatile uint8_t clock_guard_active; // 中断读取 HAL tick 时也要看到当前阶段。
static PlatformClockGuard_t peripheral_guard; // 所有早期外设共用同一固定起点。
static uint8_t peripheral_guard_started; // 外设启动期限已经开始。
static uint8_t peripheral_guard_finished; // 同一次上电不能重新开始总期限。
static volatile uint8_t peripheral_guard_active; // 当前线程正在受保护的 HAL 调用中。

uint32_t PlatformClockRawTick(void)
{
    return CLOCK_STARTUP_READ_RAW_TICK(); // 诊断事件始终使用真实时基。
}

uint8_t PlatformPeripheralStartupBegin(void)
{
    const uint32_t previous = __get_PRIMASK(); // 保存调用者的中断状态。
    __disable_irq(); // 避免与中断中的时基查询交错。
    uint8_t result = 0u; // 非法阶段不修改已有期限。
    if (!clock_guard_active && !peripheral_guard_started && !peripheral_guard_finished && __get_IPSR() == 0u)
    {
        result = PlatformClockGuardBegin(&peripheral_guard, DWT->CYCCNT,
                                         PERIPHERAL_STARTUP_TOTAL_CYCLES, PERIPHERAL_STARTUP_TOTAL_POLLS);
        if (result) peripheral_guard_started = 1u; // 成功后才开放后续受保护调用。
    }
    __set_PRIMASK(previous); // 立即恢复原中断状态。
    return result;
}

uint8_t PlatformPeripheralStartupEnter(void)
{
    const uint32_t previous = __get_PRIMASK(); // 保留调用者状态。
    __disable_irq(); // 检查和发布保护状态时禁止中断打断。
    uint8_t result = 0u; // 默认拒绝无效或嵌套调用。
    if (peripheral_guard_started && !peripheral_guard_active && __get_IPSR() == 0u)
    {
        (void)PlatformClockGuardTickStep(&peripheral_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT,
                                         PERIPHERAL_EXPIRED_STEP_MS); // 进入前计入同一个期限。
        if (!peripheral_guard.expired) { peripheral_guard_active = 1u; result = 1u; } // 未到期才允许 HAL 操作。
    }
    __set_PRIMASK(previous); // 不延长中断屏蔽时间。
    return result;
}

uint8_t PlatformPeripheralStartupLeave(void)
{
    const uint32_t previous = __get_PRIMASK(); // 保存外部屏蔽状态。
    __disable_irq(); // 保护到期状态和开关的更新。
    uint8_t result = 0u; // 未进入时不能报告成功。
    if (peripheral_guard_started && peripheral_guard_active && __get_IPSR() == 0u)
    {
        (void)PlatformClockGuardTickStep(&peripheral_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT,
                                         PERIPHERAL_EXPIRED_STEP_MS); // 即使 HAL 未查询时钟也检查已用时间。
        result = (uint8_t)!peripheral_guard.expired; // 到期后不能接受本次操作成功。
        peripheral_guard_active = 0u; // 后续普通代码恢复读取真实时基。
    }
    __set_PRIMASK(previous); // 恢复调用前状态。
    return result;
}

uint8_t PlatformPeripheralStartupCheck(void)
{
    const uint32_t previous = __get_PRIMASK(); // 保存调用者的中断状态。
    __disable_irq(); // 与 HAL 时间查询共用同一个期限状态。
    uint8_t result = 0u; // 非法上下文和未进入保护时拒绝继续等待。
    if (peripheral_guard_started && peripheral_guard_active && __get_IPSR() == 0u)
    {
        (void)PlatformClockGuardTickStep(&peripheral_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT,
                                         PERIPHERAL_EXPIRED_STEP_MS); // 累计检查次数并判断周期。
        result = (uint8_t)!peripheral_guard.expired; // 超期后本次上电不可恢复为有效。
    }
    __set_PRIMASK(previous); // 每次轮询后恢复原中断状态。
    return result;
}

uint8_t PlatformPeripheralStartupFinish(void)
{
    const uint32_t previous = __get_PRIMASK(); // 保留外部中断状态。
    __disable_irq(); // 最终检查与结束必须连续。
    uint8_t result = 0u; // 未开始或仍在保护内部不能结束。
    if (peripheral_guard_started && !peripheral_guard_active && __get_IPSR() == 0u)
    {
        (void)PlatformClockGuardTickStep(&peripheral_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT,
                                         PERIPHERAL_EXPIRED_STEP_MS); // 最后一次检查仍使用原始起点。
        result = (uint8_t)!peripheral_guard.expired; // 不把期限外的结果当作成功。
        peripheral_guard_started = 0u; // 结束本次启动期限。
        peripheral_guard_finished = 1u; // 本次上电不能再次开始以延长总期限。
    }
    __set_PRIMASK(previous); // 恢复原中断状态。
    return result;
}

void PlatformClockStartupBegin(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用周期计数所在的调试模块。
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 保持已有计数，不假设上次值为零。
    (void)PlatformClockGuardBegin(&clock_guard, DWT->CYCCNT,
                                  CLOCK_STARTUP_TOTAL_CYCLES, CLOCK_STARTUP_TOTAL_POLLS);
    __DMB(); // 状态写入完成后再向 HAL 查询发布保护。
    clock_guard_active = 1u; // 后续 HAL 时间查询使用本次固定期限。
}

/* 覆盖 HAL 的弱实现；正常运行时直接读取原有毫秒计数。 */
uint32_t HAL_GetTick(void)
{
    if (!clock_guard_active && (!peripheral_guard_active || __get_IPSR() != 0u))
        return CLOCK_STARTUP_READ_RAW_TICK(); // 外设保护之外及中断中始终读取真实时基。
    const uint32_t previous = __get_PRIMASK(); // 保留调用方的中断状态。
    __disable_irq(); // 防止任务和中断同时改写固定期限状态。
    const uint32_t result = clock_guard_active
        ? PlatformClockGuardTick(&clock_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT)
        : PlatformClockGuardTickStep(&peripheral_guard, CLOCK_STARTUP_READ_RAW_TICK(), DWT->CYCCNT,
                                     PERIPHERAL_EXPIRED_STEP_MS); // 仅启动线程的受保护调用使用临时时间。
    __set_PRIMASK(previous); // 查询结束立即恢复原中断状态。
    return result; // 仅在启动期限用尽时给 HAL 返回临时偏移。
}

static uint8_t ClockVosReady(void *context)
{
    (void)context; // 电压状态由本板 PWR 寄存器提供。
    return (uint8_t)(__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) != 0u); // 全部就绪才允许切换时钟。
}

static uint32_t ClockReadCycles(void *context)
{
    (void)context; // 周期来源由本板固定。
    return DWT->CYCCNT; // 计数停止时由检查次数上限兜底。
}

uint8_t PlatformClockStartupWaitVos(void)
{
    return (uint8_t)(PlatformStartupWait(ClockVosReady, ClockReadCycles, NULL,
                                        CLOCK_STARTUP_VOS_CYCLES, CLOCK_STARTUP_VOS_POLLS) ==
                     STARTUP_WAIT_READY); // 两种期限内就绪才报告成功。
}

uint8_t PlatformClockStartupFinish(void)
{
    const uint32_t initial_tick = CLOCK_STARTUP_READ_RAW_TICK(); // 检查切换后的 HAL 时基是否继续工作。
    const uint32_t tick_started_cycles = DWT->CYCCNT; // 固定本次时基检查的起点。
    uint8_t tick_running = 0u; // 尚未观察到真实毫秒计数前进。
    for (uint32_t poll = 0u; poll < CLOCK_STARTUP_TICK_POLLS; ++poll)
    {
        const uint32_t previous = __get_PRIMASK(); // 保存本轮查询前的中断状态。
        __disable_irq(); // 与中断中的 HAL_GetTick 共用同一个总期限状态。
        const uint32_t raw_tick = CLOCK_STARTUP_READ_RAW_TICK(); // 不使用 HAL 的临时偏移。
        const uint32_t cycles = DWT->CYCCNT; // 同时读取真实周期计数。
        (void)PlatformClockGuardTick(&clock_guard, raw_tick, cycles); // 本轮也计入整个启动总期限。
        const uint8_t expired = clock_guard.expired; // 保存总期限检查结果。
        __set_PRIMASK(previous); // 每次检查后立即恢复原中断状态。
        if (expired || (uint32_t)(cycles - tick_started_cycles) >= CLOCK_STARTUP_TICK_CYCLES)
            break; // 总期限或本轮周期期限到达后不能接受就绪。
        if (raw_tick != initial_tick) // 只接受期限内真实发生的毫秒计数变化。
        {
            tick_running = 1u; // 记录原始时基已经前进。
            break; // 不必继续等待其他 tick。
        }
    }
    const uint32_t previous = __get_PRIMASK(); // 防止中断同时查询临时 HAL tick。
    __disable_irq(); // 保护到期状态读取和保护退出。
    const uint32_t last_cycles = DWT->CYCCNT; // 把检查结束前的执行耗时也计入总期限。
    const uint8_t within_total = (uint8_t)((uint32_t)(last_cycles - clock_guard.started_cycles) <
                                           clock_guard.cycle_limit); // 从本次启动的原始起点计算。
    const uint8_t within_tick_wait = (uint8_t)((uint32_t)(last_cycles - tick_started_cycles) <
                                               CLOCK_STARTUP_TICK_CYCLES); // 本地检查也不能超过期限。
    const uint8_t success = (uint8_t)(tick_running && !clock_guard.expired && within_total && within_tick_wait); // 所有期限与原始时基都必须有效。
    clock_guard_active = 0u; // 后续 HAL 调用恢复原始毫秒时基。
    __set_PRIMASK(previous); // 恢复调用前中断状态。
    return success; // 到期后即使 HAL 报告成功也不能继续启动。
}

void PlatformClockStartupFail(PlatformClockFailure_t failure)
{
    if (failure < PLATFORM_CLOCK_FAILURE_SUPPLY || failure > PLATFORM_CLOCK_FAILURE_WATCHDOG)
        failure = PLATFORM_CLOCK_FAILURE_DEADLINE; // 无效原因映射为通用期限失败，不写越界编号。
    if (clock_guard_active && clock_guard.expired)
        failure = PLATFORM_CLOCK_FAILURE_DEADLINE; // HAL 因总期限退出时保留停表或超期原因。
    clock_guard_active = 0u; // 诊断使用原始 HAL tick，不继续产生临时偏移。
    __disable_irq(); // 关断和复位记录不得被普通中断打断。
    BSP_SafetyStartupLatchOutputsOff(); // 不依赖 HAL 或任务，立即关闭受控输出。
    const uint8_t watchdog_failure = (uint8_t)(failure == PLATFORM_CLOCK_FAILURE_WATCHDOG); // 区分看门狗与时钟故障。
    const InitDiagnosticStage_e stage = (failure == PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE || watchdog_failure)
                                            ? INIT_STAGE_BSP : INIT_STAGE_CLOCK; // 区分外设和主时钟期限。
    InitDiagnosticReportFailure(stage, watchdog_failure ? INIT_MODULE_WATCHDOG : INIT_MODULE_CLOCK,
                                watchdog_failure ? INIT_FAILURE_WATCHDOG_CONFIG : INIT_FAILURE_CLOCK,
                                watchdog_failure ? INIT_RESOURCE_WATCHDOG_STARTUP : (uint16_t)failure); // 保存具体失败位置。
    BSP_ResetRecoveryMarkFault((ResetRecoveryCause_e)(RESET_RECOVERY_CAUSE_CLOCK_SUPPLY +
                                                     (uint32_t)failure - 1u)); // 跨复位保存具体原因。
    if (!BSP_ResetRecoveryIsLockedOut()) // 恢复额度未用尽时才允许自动复位。
    {
        __HAL_RCC_CLEAR_RESET_FLAGS(); // 防止旧 POR 标志在软件复位后误清恢复次数。
        NVIC_SystemReset(); // 让下一次启动重新检查全部核心条件。
    }
    for (;;)
    {
        IWDG1->KR = 0xAAAAu; // 若看门狗跨复位仍在运行，防止锁定状态再次触发复位。
        __NOP(); // 已锁定或复位意外返回时保持关断，不继续启动。
    }
}
