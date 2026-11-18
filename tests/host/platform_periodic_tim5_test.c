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
static int first_task, second_task;
static TaskHandle_t current = &first_task;
static unsigned critical_depth, notifications, waits;
static uint32_t arm_race_tick; // 设置非零时模拟计数在写比较寄存器前到期。
static uint8_t irq_before_wait, timeout, early_signal;
static uint32_t delay_us; // 模拟 ISR 之后调度任务的延迟。
static uint32_t reset_tick; // 模拟计数起点接近回绕。
static unsigned deleted_signals; // 确认资源释放顺序。

void __HAL_RCC_TIM5_FORCE_RESET(void) { test_timer = (TestTimer_t){.CNT = reset_tick}; }
void __HAL_RCC_TIM5_RELEASE_RESET(void) {}
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
void NVIC_ClearPendingIRQ(int irq) { TEST_CHECK(irq == TIM5_IRQn); }
uint8_t PlatformIrqIsInHandler(void) { return in_handler; }
BaseType_t xTaskGetSchedulerState(void) { return taskSCHEDULER_RUNNING; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current; }
void taskENTER_CRITICAL(void) { ++critical_depth; }

static void FireCompare(void)
{
    TEST_CHECK(irq_enabled && test_timer_enabled && critical_depth == 0u);
    in_handler = 1u;
    TIM5_IRQHandler(); // 无标志的额外入口不能取消有效的比较。
    TEST_CHECK(TIM5->DIER == TIM_DIER_CC1IE);
    TIM5->CNT = TIM5->CCR1 + delay_us; // 比较后可能经过多个周期才轮到任务。
    TIM5->SR = TIM_SR_CC1IF;
    TIM5_IRQHandler();
    TIM5_IRQHandler(); // 同一事件只产生一次通知。
    in_handler = 0u;
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
    TEST_CHECK(signal != NULL && !irq_enabled && !test_timer_enabled); // 确认 ISR 不会再引用此对象。
    ++deleted_signals;
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t signal, BaseType_t *reschedule)
{
    TEST_CHECK(in_handler && signal->value == 0u && TIM5->DIER == 0u);
    ++notifications;
    signal->value = 1u;
    *reschedule = pdTRUE;
    return pdTRUE;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t signal, TickType_t ticks)
{
    if (ticks != 0u)
    {
        TEST_CHECK(!critical_depth && ticks == 2u); // 亚毫秒测试等待用有限的 RTOS 超时兜底。
        ++waits;
        if (timeout) return pdFALSE; // 模拟停表或丢失比较中断。
        if (early_signal) return pdTRUE; // 模拟提前通知，计数还没有到期。
        if (!signal->value) FireCompare(); // 没有预先通知时模拟等待期间触发 ISR。
    }
    const BaseType_t ready = signal->value ? pdTRUE : pdFALSE;
    signal->value = 0u;
    return ready;
}

int main(void)
{
    const PlatformPeriodicConfig_t config = {.period_us = 500u, .budget_us = 200u}; // 测试期间使用独立亚毫秒拍点。
    PlatformPeriodicClock_t clock = {.next_tick = 17u}, other;
    PlatformPeriodicRelease_t release = {.observed_tick = 17u};
    pclk = 120000001u; // 无法精确选择 1 MHz 分频。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    pclk = 120000000u;
    RCC->CFGR = RCC_CFGR_TIMPRE; // 不支持的定时器时钟选择不能静默降级。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    RCC->CFGR = 0u;
    priority_group = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_CONFIG);
    priority_group = NVIC_PRIORITYGROUP_4;
    test_timer_enabled = 1u; // 不能重置已使用的定时器。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE && test_timer_enabled);
    test_timer_enabled = 0u;
    irq_enabled = 1u; // 也不能接管已经启用的 IRQ。
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE && irq_enabled);
    irq_enabled = 0u;
    deny_alloc = 1u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(clock.next_tick == 17u && !test_timer_enabled); // 所有失败都保留调用者和外设状态。
    deny_alloc = 0u;
    TEST_CHECK(PlatformTaskHighResOpen(&clock, &config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(irq_enabled && TIM5->PSC == 239u && TIM5->ARR == UINT32_MAX && TIM5->CR1 == TIM_CR1_CEN);
    in_handler = 1u;
    TEST_CHECK(PlatformTaskHighResWait(&clock, &release) == PLATFORM_PERIODIC_CONTEXT);
    in_handler = 0u;
    current = &second_task; // 资源不能被另一任务使用或释放。
    TEST_CHECK(PlatformTaskHighResOpen(&other, &config) == PLATFORM_PERIODIC_RESOURCE);
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
    return 0;
}
