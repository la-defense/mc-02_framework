/** @file platform_periodic_tim5_test.c
 * @brief 用寄存器和 RTOS 替身验证生产 TIM5 唤醒后端的时序和故障边界。
 */
#include "platform_periodic.h"
#include "platform_irq.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "test_check.h"
#include <stddef.h>

void TIM5_IRQHandler(void);
extern volatile uint32_t mc02_tim5_irq_notifications;

TestTimer_t test_timer;
TestRcc_t test_rcc;
uint8_t test_timer_enabled;
static uint32_t pclk = 120000000u; // 模拟当前板卡的 APB1 时钟。
static uint32_t priority_group = NVIC_PRIORITYGROUP_4;
static uint8_t irq_enabled, in_handler, deny_alloc;
static int first_task, second_task, extra_tasks[3];
static TaskHandle_t current = &first_task;
static unsigned critical_depth, notifications, waits;
static uint32_t arm_race_tick; // 设置非零时模拟计数在写比较寄存器前到期。
static uint8_t irq_before_wait, timeout, early_signal;
static uint8_t stop_after_signal, gate_before_arm, gate_while_waiting;
static uint32_t delay_us; // 模拟 ISR 之后调度任务的延迟。
static uint32_t reset_tick; // 模拟计数起点接近回绕。
static unsigned deleted_signals; // 确认资源释放顺序。
static unsigned timer_resets, nvic_clears;
static PlatformPeriodicClock_t *nested_clock; // 当前任务阻塞时，替身可切换到另一个任务。
static uint8_t nested_timeout; // 第二个任务单独超时，首个任务仍应收到通知。

void __HAL_RCC_TIM5_FORCE_RESET(void)
{
    ++timer_resets;
    test_timer = (TestTimer_t){.CNT = reset_tick};
}
void __HAL_RCC_TIM5_RELEASE_RESET(void) {}
void TestWriteTimerStatus(volatile uint32_t *reg, uint32_t value)
{
    TEST_CHECK(test_timer_enabled && reg == &TIM5->SR); // 外设时钟门关闭后不能再写 SR。
    *reg &= value; // SR 写入 1 时不能清除已有标志。
}
void __DSB(void)
{
    if (arm_race_tick != 0u)
    {
        TIM5->CNT = arm_race_tick; // 模拟任务检查未来拍点后，计数已经到期。
        arm_race_tick = 0u;
    }
}
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config, uint32_t *latency)
{ config->APB1CLKDivider = 4u; *latency = 0u; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk; }
uint32_t HAL_NVIC_GetPriorityGrouping(void) { return priority_group; }
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub)
{ TEST_CHECK(irq == TIM5_IRQn && preempt == configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY && sub == 0u); }
void HAL_NVIC_EnableIRQ(int irq) { TEST_CHECK(irq == TIM5_IRQn); irq_enabled = 1u; }
void HAL_NVIC_DisableIRQ(int irq) { TEST_CHECK(irq == TIM5_IRQn); irq_enabled = 0u; }
uint32_t NVIC_GetEnableIRQ(int irq) { TEST_CHECK(irq == TIM5_IRQn); return irq_enabled; }
void NVIC_ClearPendingIRQ(int irq) { TEST_CHECK(irq == TIM5_IRQn); ++nvic_clears; }
uint8_t PlatformIrqIsInHandler(void) { return in_handler; }
BaseType_t xTaskGetSchedulerState(void) { return taskSCHEDULER_RUNNING; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current; }
void taskENTER_CRITICAL(void) { ++critical_depth; }

static void RunIrq(void)
{
    TEST_CHECK(irq_enabled && test_timer_enabled && critical_depth == 0u);
    in_handler = 1u;
    TIM5_IRQHandler();
    TIM5_IRQHandler(); // 同一事件只产生一次通知。
    in_handler = 0u;
}

static void FireCompare(void)
{
    const uint32_t enabled = TIM5->DIER; // 单通道场景进入时，其他持有者没有正在等待的比较。
    RunIrq(); // 无标志的额外入口不能取消有效的比较。
    TEST_CHECK(TIM5->DIER == enabled);
    switch (enabled)
    {
        case TIM_DIER_CC1IE:
            TIM5->CNT = TIM5->CCR1 + delay_us;
            TIM5->SR |= TIM_SR_CC1IF;
            break;
        case TIM_DIER_CC2IE:
            TIM5->CNT = TIM5->CCR2 + delay_us;
            TIM5->SR |= TIM_SR_CC2IF;
            break;
        case TIM_DIER_CC3IE:
            TIM5->CNT = TIM5->CCR3 + delay_us;
            TIM5->SR |= TIM_SR_CC3IF;
            break;
        case TIM_DIER_CC4IE:
            TIM5->CNT = TIM5->CCR4 + delay_us;
            TIM5->SR |= TIM_SR_CC4IF;
            break;
        default: TEST_CHECK(0); // 多通道同时到期由单独的测试步骤模拟。
    }
    RunIrq();
}

void taskEXIT_CRITICAL(void)
{
    TEST_CHECK(critical_depth != 0u);
    --critical_depth;
    if (!critical_depth && irq_before_wait && (TIM5->DIER & TIM_DIER_CC1IE))
    {
        irq_before_wait = 0u; // ISR 先于实际阻塞时，通知应保存在信号量中。
        FireCompare();
    }
}
void portYIELD_FROM_ISR(BaseType_t reschedule) { TEST_CHECK(in_handler && reschedule); }
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *storage)
{
    storage->value = 0u;
    return deny_alloc ? NULL : storage;
}
void vSemaphoreDelete(SemaphoreHandle_t signal)
{
    TEST_CHECK(signal != NULL && critical_depth != 0u); // 释放期间 ISR 无法访问对象。
    ++deleted_signals;
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t signal, BaseType_t *reschedule)
{
    TEST_CHECK(in_handler && signal != NULL && signal->value == 0u);
    ++notifications;
    signal->value = 1u;
    *reschedule = pdTRUE;
    return pdTRUE;
}

static void RunOtherTaskWhileBlocked(void)
{
    TEST_CHECK(TIM5->DIER == TIM_DIER_CC1IE);
    PlatformPeriodicClock_t *clock = nested_clock; // 取走模拟切换请求，避免嵌套等待再次触发。
    nested_clock = NULL;
    const unsigned clears = nvic_clears; // 普通比较和部分释放都不能清掉共享的 NVIC 请求。
    current = &second_task; // 让第二个任务在同一计数域等下一拍。
    PlatformPeriodicRelease_t release = {.scheduled_tick = 17u};
    const PlatformPeriodicResult_t result = PlatformTaskHighResWait(clock, &release);
    if (nested_timeout) // 第二个任务超时并退出时，第一个通道仍有待处理的通知。
    {
        TEST_CHECK(result == PLATFORM_PERIODIC_CLOCK && release.scheduled_tick == 17u);
        TEST_CHECK(PlatformTaskHighResClose(clock) == PLATFORM_PERIODIC_READY);
        TEST_CHECK(TIM5->DIER == TIM_DIER_CC1IE && TIM5->SR == TIM_SR_CC1IF);
        TEST_CHECK(irq_enabled && test_timer_enabled);
        RunIrq(); // 第一拍留在 NVIC 和外设中的请求仍然可以交付。
    }
    else
    {
        TEST_CHECK(result == PLATFORM_PERIODIC_READY && release.scheduled_tick == TIM5->CCR2);
    }
    TEST_CHECK(nvic_clears == clears);
    current = &first_task; // 恢复原任务，消费自己的信号量。
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t signal, TickType_t ticks)
{
    if (ticks == 0u && gate_before_arm)
        test_timer_enabled = 0u; // 模拟清旧通知期间另一个任务关闭时钟。
    if (ticks != 0u)
    {
        TEST_CHECK(!critical_depth && ticks == 2u); // 亚毫秒测试等待用有限的 RTOS 超时兜底。
        ++waits;
        if (gate_while_waiting)
        {
            test_timer_enabled = 0u; // 已打开比较中断，计数器此后无法通知。
            return pdFALSE;
        }
        if (timeout) return pdFALSE; // 模拟停表或丢失比较中断。
        if (early_signal) return pdTRUE; // 模拟提前通知，计数还没有到期。
        if (!signal->value && nested_clock != NULL)
            RunOtherTaskWhileBlocked(); // 原任务阻塞，允许另一个任务设置自己的比较。
        if (!signal->value && current == &second_task && (TIM5->DIER & TIM_DIER_CC2IE))
        {
            TEST_CHECK(TIM5->DIER == (TIM_DIER_CC1IE | TIM_DIER_CC2IE)); // 两个任务的比较应同时有效。
            TIM5->CNT = TIM5->CCR1; // 两个任务的同一拍在调度前已经到期。
            TEST_CHECK(TIM5->CNT == TIM5->CCR2);
            TIM5->SR |= TIM_SR_CC1IF; // 首个任务的比较已经送到共享中断。
            if (nested_timeout) return pdFALSE; // 仅第二个任务失去唤醒，不应清除首个任务的标志。
            TIM5->SR |= TIM_SR_CC2IF;
            RunIrq(); // 一次 IRQ 为两个通道分别交付唤醒。
        }
        if (!signal->value) FireCompare(); // 没有预先通知时模拟等待期间触发 ISR。
        if (stop_after_signal)
            TIM5->CR1 = 0u; // 已收到通知，等待任务恢复时计数器却停了。
    }
    const BaseType_t ready = signal->value ? pdTRUE : pdFALSE;
    signal->value = 0u;
    return ready;
}

static void TestTimerAvailability(const PlatformPeriodicConfig_t *config)
{
    PlatformPeriodicClock_t clock;
    PlatformPeriodicRelease_t release = {.scheduled_tick = 17u};
    reset_tick = 0u;
    current = &first_task;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, config) == PLATFORM_PERIODIC_READY);
    const unsigned initial_waits = waits;
    TIM5->CR1 = 0u; // 第一拍即使已到期，关停计数器后也不能领取。
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    TEST_CHECK(clock.next_tick == 0u && release.scheduled_tick == 17u && waits == initial_waits);
    TIM5->CR1 = TIM_CR1_CEN;
    irq_enabled = 0u; // 后续拍不会有唤醒，立即报告错误。
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    TEST_CHECK(clock.next_tick == 0u && release.scheduled_tick == 17u);
    irq_enabled = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_READY);
    gate_before_arm = 1u; // 领取首拍后，下一拍仍在未来。
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    gate_before_arm = 0u;
    TEST_CHECK(waits == initial_waits && clock.next_tick == 500u && release.scheduled_tick == 0u);
    test_timer_enabled = 1u; // 重新给寄存器供时钟，继续验证醒来路径。
    stop_after_signal = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    stop_after_signal = 0u;
    TEST_CHECK(TIM5->DIER == 0u && clock.next_tick == 500u && release.scheduled_tick == 0u);
    TIM5->CNT = 0u; // 再试同一拍，进入有限阻塞后关掉外设时钟。
    TIM5->CR1 = TIM_CR1_CEN;
    gate_while_waiting = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    gate_while_waiting = 0u;
    TEST_CHECK(clock.next_tick == 500u && release.scheduled_tick == 0u);
    const unsigned clears = nvic_clears;
    TEST_CHECK(PlatformTaskHighResClose(&clock) == PLATFORM_PERIODIC_READY && !irq_enabled);
    TEST_CHECK(!test_timer_enabled && nvic_clears == clears + 1u); // 仍可撤销 NVIC 请求并释放任务资源。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, config) == PLATFORM_PERIODIC_READY && TIM5->DIER == 0u);
    TEST_CHECK(PlatformTaskHighResClose(&clock) == PLATFORM_PERIODIC_READY);
}

static void TestGatedChannelRelease(const PlatformPeriodicConfig_t *config)
{
    PlatformPeriodicClock_t first, second;
    PlatformPeriodicRelease_t release;
    current = &first_task;
    TEST_CHECK(PlatformTaskHighResOpen(&first, config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_READY);
    current = &second_task;
    TEST_CHECK(PlatformTaskHighResOpen(&second, config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskHighResWait(&second, &release) == PLATFORM_PERIODIC_READY);

    current = &first_task;
    gate_while_waiting = 1u; // 阻塞期间关掉外设时钟，无法立即清理通道 1 的比较寄存器。
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_CLOCK);
    gate_while_waiting = 0u;
    TIM5->SR |= TIM_SR_CC1IF; // 模拟关时钟前已送出的中断请求。
    in_handler = 1u;
    TIM5_IRQHandler(); // 时钟仍关闭，ISR 不得访问外设状态或通知任务。
    in_handler = 0u;
    const unsigned clears = nvic_clears;
    const unsigned delivered = notifications;
    TEST_CHECK(PlatformTaskHighResClose(&first) == PLATFORM_PERIODIC_READY && irq_enabled && nvic_clears == clears);

    test_timer_enabled = 1u; // 恢复时钟，硬件仍保留已释放通道的比较请求。
    TIM5->CNT = 500u;
    RunIrq();
    TEST_CHECK(TIM5->DIER == 0u && TIM5->SR == 0u && notifications == delivered); // 清旧请求，不能通知已释放的对象。
    current = &second_task;
    TEST_CHECK(PlatformTaskHighResWait(&second, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 500u);
    TEST_CHECK(PlatformTaskHighResClose(&second) == PLATFORM_PERIODIC_READY);
}

static void TestSharedTimer(const PlatformPeriodicConfig_t *config)
{
    PlatformPeriodicClock_t first, second, extra[3];
    PlatformPeriodicRelease_t release;
    const unsigned original_resets = timer_resets;
    const unsigned original_deletes = deleted_signals;
    reset_tick = 0u;
    current = &first_task;
    TEST_CHECK(PlatformTaskHighResOpen(&first, config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 0u);
    TIM5->CNT = 10u; // 先使用持有者的计数检查本轮完成。
    PlatformPeriodicTiming_t timing = {0};
    TEST_CHECK(PlatformTaskHighResFinish(&first, &release, &timing) == PLATFORM_PERIODIC_READY && timing.work_ticks == 10u);
    TIM5->CR1 = 0u; // 模拟完成前计数器被关闭，原输出必须保留。
    TEST_CHECK(PlatformTaskHighResFinish(&first, &release, &timing) == PLATFORM_PERIODIC_CLOCK && timing.work_ticks == 10u);
    TIM5->CR1 = TIM_CR1_CEN;
    irq_enabled = 0u; // 计数仍前进，但关闭比较 IRQ 后不能报告本拍正常完成。
    TEST_CHECK(PlatformTaskHighResFinish(&first, &release, &timing) == PLATFORM_PERIODIC_CLOCK && timing.work_ticks == 10u);
    irq_enabled = 1u;

    current = &second_task;
    TIM5->CNT = 40u; // 原有计数不能因另一任务申请而回到零。
    TEST_CHECK(PlatformTaskHighResFinish(&first, &release, &timing) == PLATFORM_PERIODIC_CONTEXT && timing.work_ticks == 10u);
    deny_alloc = 1u;
    TEST_CHECK(PlatformTaskHighResOpen(&second, config) == PLATFORM_PERIODIC_RESOURCE && TIM5->CNT == 40u);
    deny_alloc = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&second, config) == PLATFORM_PERIODIC_READY && second.next_tick == 40u);
    TEST_CHECK(timer_resets == original_resets + 1u);
    TEST_CHECK(PlatformTaskHighResClose(&second) == PLATFORM_PERIODIC_READY && TIM5->CNT == 40u);

    reset_tick = 100u; // 重新建立同一拍点，模拟两个任务同时到期。
    current = &first_task;
    TEST_CHECK(PlatformTaskHighResClose(&first) == PLATFORM_PERIODIC_READY && !irq_enabled);
    TEST_CHECK(PlatformTaskHighResOpen(&first, config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_READY);
    current = &second_task;
    const unsigned shared_resets = timer_resets;
    TEST_CHECK(PlatformTaskHighResOpen(&second, config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskHighResWait(&second, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(timer_resets == shared_resets);

    current = &first_task;
    nested_clock = &second;
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 600u);
    TEST_CHECK(mc02_tim5_irq_notifications == 2u && TIM5->DIER == 0u); // 一次 ISR 交付两个独立通知。
    nested_timeout = 1u;
    nested_clock = &second;
    TEST_CHECK(PlatformTaskHighResWait(&first, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 1100u);
    TEST_CHECK(mc02_tim5_irq_notifications == 3u && TIM5->DIER == 0u); // 另一个通道超时并退出后仍收到通知。
    nested_timeout = 0u;

    // 复用第二个通道，四个持有者用满全部通道后，第五个仍应失败。
    current = &second_task;
    TEST_CHECK(PlatformTaskHighResOpen(&second, config) == PLATFORM_PERIODIC_READY && second.next_tick == 1100u);
    for (size_t i = 0; i < 3u; ++i)
    {
        current = &extra_tasks[i];
        const PlatformPeriodicResult_t expected = i < 2u ? PLATFORM_PERIODIC_READY : PLATFORM_PERIODIC_RESOURCE;
        TEST_CHECK(PlatformTaskHighResOpen(&extra[i], config) == expected);
    }
    // 释放通道 3 后，第五个任务可使用该通道，剩余任务仍保持原计数。
    current = &extra_tasks[0];
    TEST_CHECK(PlatformTaskHighResClose(&extra[0]) == PLATFORM_PERIODIC_READY);
    current = &extra_tasks[2];
    TEST_CHECK(PlatformTaskHighResOpen(&extra[2], config) == PLATFORM_PERIODIC_READY && extra[2].next_tick == 1100u);
    TEST_CHECK(PlatformTaskHighResWait(&extra[2], &release) == PLATFORM_PERIODIC_READY); // 领取通道 3 的首拍。
    current = &extra_tasks[1];
    TEST_CHECK(PlatformTaskHighResWait(&extra[1], &release) == PLATFORM_PERIODIC_READY); // 领取通道 4 的首拍。
    current = &extra_tasks[2];
    TEST_CHECK(PlatformTaskHighResWait(&extra[2], &release) == PLATFORM_PERIODIC_READY && TIM5->CCR3 == 1600u);
    TEST_CHECK(PlatformTaskHighResClose(&extra[2]) == PLATFORM_PERIODIC_READY && irq_enabled);
    current = &extra_tasks[1];
    TEST_CHECK(PlatformTaskHighResWait(&extra[1], &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 1600u);
    TEST_CHECK(PlatformTaskHighResWait(&extra[1], &release) == PLATFORM_PERIODIC_READY && TIM5->CCR4 == 2100u);
    TEST_CHECK(mc02_tim5_irq_notifications == 5u); // 两个高号通道也分别收到比较通知。
    TEST_CHECK(PlatformTaskHighResClose(&extra[1]) == PLATFORM_PERIODIC_READY && irq_enabled);
    current = &second_task;
    TEST_CHECK(PlatformTaskHighResClose(&second) == PLATFORM_PERIODIC_READY && irq_enabled);
    current = &first_task;
    TEST_CHECK(PlatformTaskHighResClose(&first) == PLATFORM_PERIODIC_READY && !irq_enabled && !test_timer_enabled);
    TEST_CHECK(deleted_signals == original_deletes + 8u && timer_resets == shared_resets);
}

int main(void)
{
    const PlatformPeriodicConfig_t config = {.period_us = 500u, .budget_us = 200u}; // 测试期间使用独立亚毫秒拍点。
    PlatformPeriodicClock_t clock = {.next_tick = 17u}, other;
    PlatformPeriodicRelease_t release = {.observed_tick = 17u};
    pclk = 120000001u; // 无法精确选择 1 MHz 分频。
    TEST_CHECK(PlatformTaskHighResAvailableChannels() == 0u);
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    pclk = 120000000u;
    RCC->CFGR = RCC_CFGR_TIMPRE; // 不支持的定时器时钟选择不能静默降级。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    RCC->CFGR = 0u;
    priority_group = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    priority_group = NVIC_PRIORITYGROUP_4;
    test_timer_enabled = 1u; // 不能重置已使用的定时器。
    TEST_CHECK(PlatformTaskHighResAvailableChannels() == 0u);
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE && test_timer_enabled);
    test_timer_enabled = 0u;
    irq_enabled = 1u; // 也不能接管已经启用的 IRQ。
    TEST_CHECK(PlatformTaskHighResAvailableChannels() == 0u);
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE && irq_enabled);
    irq_enabled = 0u;
    TEST_CHECK(PlatformTaskHighResAvailableChannels() == 4u && !test_timer_enabled && !irq_enabled);
    deny_alloc = 1u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(clock.next_tick == 17u && !test_timer_enabled); // 所有失败都保留调用者和外设状态。
    deny_alloc = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(irq_enabled && TIM5->PSC == 239u && TIM5->ARR == UINT32_MAX && TIM5->CR1 == TIM_CR1_CEN);
    in_handler = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CONTEXT);
    in_handler = 0u;
    // 同一任务不得重复申请，另一个任务不能用旧状态地址取得通道。
    TEST_CHECK(PlatformTaskHighResOpen(&other, &config) == PLATFORM_PERIODIC_RESOURCE);
    current = &second_task;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CONTEXT);
    TEST_CHECK(PlatformTaskHighResClose(&clock) == PLATFORM_PERIODIC_CONTEXT);
    current = &first_task;

    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 0u);
    irq_before_wait = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 500u);
    TEST_CHECK(waits == 1u && notifications == 1u);
    delay_us = 1001u; // 处理延迟跨越两个完整周期。
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(release.scheduled_tick == 2000u && release.observed_tick == 2001u && release.skipped == 2u);
    arm_race_tick = 2500u; // 硬件已到期时不等待一个可能错过的中断。
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_READY && waits == 2u);
    TEST_CHECK(release.scheduled_tick == 2500u);
    early_signal = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK && TIM5->DIER == 0u);
    early_signal = 0u;
    timeout = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK && TIM5->DIER == 0u);
    TEST_CHECK(clock.next_tick == 3000u && release.scheduled_tick == 2500u); // 失败不推进任务周期。
    TEST_CHECK(PlatformTaskHighResClose(&clock) == PLATFORM_PERIODIC_READY && !test_timer_enabled && !irq_enabled);
    TEST_CHECK(mc02_tim5_irq_notifications == 2u && deleted_signals == 1u);
    TEST_CHECK(PlatformTaskHighResClose(&clock) == PLATFORM_PERIODIC_CONTEXT);
    current = &second_task;
    reset_tick = UINT32_MAX - 250u; // 从回绕前的拍点开始新一次申请。
    timeout = 0u;
    delay_us = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&other, &config) == PLATFORM_PERIODIC_READY); // 释放后另一任务可重新取得资源。
    TEST_CHECK(mc02_tim5_irq_notifications == 0u);
    TEST_CHECK(PlatformTaskHighResWait(&other, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == reset_tick);
    TEST_CHECK(PlatformTaskHighResWait(&other, &release) == PLATFORM_PERIODIC_READY && release.scheduled_tick == 249u);
    TEST_CHECK(mc02_tim5_irq_notifications == 1u);
    TEST_CHECK(PlatformTaskHighResClose(&other) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(deleted_signals == 2u);
    TestSharedTimer(&config);
    TestTimerAvailability(&config);
    TestGatedChannelRelease(&config);
    return 0;
}
