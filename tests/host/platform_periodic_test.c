/** @file platform_periodic_test.c
 * @brief 通过生产周期算法和 FreeRTOS 适配器验证时序边界。
 */
#include "platform_periodic.h"
#include "platform_irq.h"
#include "FreeRTOS.h"
#include "task.h"
#include "test_check.h"

#include <limits.h>
#include <stddef.h>

static TickType_t now_tick; // 由测试控制的 RTOS 计数。
static BaseType_t scheduler_state = taskSCHEDULER_RUNNING; // 默认允许任务阻塞。
static uint8_t in_handler; // 模拟当前在异常处理上下文。
static uint8_t wake_early; // 模拟等待被提前取消。
static uint32_t wake_latency; // 目标时刻之后还等待了多少刻度才取得 CPU。
static unsigned sleep_count; // 检查已经到期时不会再调用后端等待。
static TickType_t sleep_until; // 记录后端收到的绝对等待时刻。

uint8_t PlatformIrqIsInHandler(void) { return in_handler; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler_state; }
TickType_t xTaskGetTickCount(void) { return now_tick; }

void vTaskDelayUntil(TickType_t *previous_wake, TickType_t increment)
{
    TEST_CHECK(*previous_wake == now_tick && increment != 0u);
    ++sleep_count;
    *previous_wake += increment; // FreeRTOS 使用上次拍点和固定间隔得到目标。
    sleep_until = *previous_wake;
    now_tick = wake_early ? sleep_until - 1u : sleep_until + wake_latency; // 注入异常提前或超时唤醒。
}

static void TestExactSetup(void)
{
    PlatformPeriodicClock_t clock = {.next_tick = 37u}; // 无效配置不能改写已有状态。
    const PlatformPeriodicConfig_t fine = {.period_us = 500u, .budget_us = 200u}; // 后端能准确表示时允许亚毫秒周期。
    const PlatformPeriodicConfig_t over = {.period_us = 5000u, .budget_us = 6000u}; // 预算不可大于周期。
    const PlatformPeriodicConfig_t half_wrap = {.period_us = UINT32_MAX, .budget_us = 1u}; // 超过半个计数范围。
    TEST_CHECK(PlatformPeriodicInit(&clock, &fine, 1000u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformPeriodicInit(&clock, &fine, 125u, 0u) == PLATFORM_PERIODIC_CONFIG); // 周期可表示但预算不可表示。
    TEST_CHECK(PlatformPeriodicInit(&clock, &over, 1u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformPeriodicInit(&clock, &half_wrap, 1u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformPeriodicInit(&clock, &fine, 0u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformPeriodicInit(&clock, NULL, 1u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformPeriodicInit(NULL, &fine, 1u, 0u) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(clock.next_tick == 37u);
    TEST_CHECK(PlatformPeriodicInit(&clock, &fine, 100u, 0u) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(clock.period_ticks == 5u && clock.budget_ticks == 2u && clock.next_tick == 0u);
}

static void TestPlan(void)
{
    const PlatformPeriodicRequest_t requests[] = {
        {{5000u, 2000u}, PLATFORM_PERIODIC_SOURCE_AUTO}, // 允许使用 1 ms 的 RTOS 刻度。
        {{500u, 200u}, PLATFORM_PERIODIC_SOURCE_AUTO}, // 周期和预算都需要硬件刻度。
        {{2000u, 250u}, PLATFORM_PERIODIC_SOURCE_AUTO}, // 整毫秒周期的预算仍可能需要硬件。
        {{3000u, 1000u}, PLATFORM_PERIODIC_SOURCE_HIGH_RES}, // 显式要求用独立比较通道。
    };
    PlatformPeriodicCapabilities_t capabilities = PlatformTaskPeriodicCapabilities();
    TEST_CHECK(capabilities.rtos_quantum_us == 1000u && capabilities.high_res_quantum_us == 0u);
    uint32_t mask = 0x80u;
    size_t failed = 99u;
    TEST_CHECK(PlatformPeriodicPlan(requests, 4u, &capabilities, &mask, &failed) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(mask == 0x80u && failed == 1u); // 无硬件支持时拒绝首个无法精确表示的任务。
    capabilities.high_res_quantum_us = 1u;
    capabilities.high_res_channels = 2u;
    TEST_CHECK(PlatformPeriodicPlan(requests, 4u, &capabilities, &mask, &failed) == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(mask == 0x80u && failed == 3u); // 已选择的前几项不能作为部分结果发布。
    capabilities.high_res_channels = 3u;
    TEST_CHECK(PlatformPeriodicPlan(requests, 4u, &capabilities, &mask, &failed) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(mask == 0x0Eu && failed == 3u); // 只有后三项使用硬件，成功不改上次错误索引。
    TEST_CHECK(PlatformPeriodicPlan(requests, 33u, &capabilities, &mask, &failed) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(mask == 0x0Eu && failed == 3u); // 拒绝超过位图容量的计划。

    const PlatformPeriodicRequest_t invalid = {{1000u, 1500u}, PLATFORM_PERIODIC_SOURCE_AUTO}; // 预算超出周期。
    TEST_CHECK(PlatformPeriodicPlan(&invalid, 1u, &capabilities, &mask, &failed) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(mask == 0x0Eu && failed == 0u);
    const PlatformPeriodicRequest_t rtos_only = {{4000u, 500u}, PLATFORM_PERIODIC_SOURCE_RTOS}; // 预算不能向整毫秒取整。
    TEST_CHECK(PlatformPeriodicPlan(&rtos_only, 1u, &capabilities, &mask, NULL) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(mask == 0x0Eu);
    TEST_CHECK(PlatformPeriodicPlan(NULL, 0u, &capabilities, &mask, NULL) == PLATFORM_PERIODIC_READY && mask == 0u);
}

static void TestScheduleAndBudget(void)
{
    PlatformPeriodicClock_t clock;
    const PlatformPeriodicConfig_t config = {.period_us = 5000u, .budget_us = 2000u}; // 计划间隔 5 ms，执行预算 2 ms。
    PlatformPeriodicRelease_t release;
    PlatformPeriodicTiming_t timing;
    TEST_CHECK(PlatformPeriodicInit(&clock, &config, 1000u, 100u) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformPeriodicPoll(&clock, 100u, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(release.scheduled_tick == 100u && release.skipped == 0u);
    TEST_CHECK(PlatformPeriodicFinish(&clock, &release, 102u, &timing) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(timing.work_ticks == 2u && !timing.over_budget && !timing.missed_deadline); // 预算恰好用完。
    TEST_CHECK(PlatformPeriodicFinish(&clock, &release, 103u, &timing) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(timing.over_budget && !timing.missed_deadline); // 单轮预算超出，但仍在计划截止之前。
    TEST_CHECK(PlatformPeriodicPoll(&clock, 104u, &release) == PLATFORM_PERIODIC_WAIT);
    TEST_CHECK(release.scheduled_tick == 105u && release.wait_ticks == 1u); // 本轮工作结束时间不会平移下一拍。
    TEST_CHECK(PlatformPeriodicPoll(&clock, 119u, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(release.scheduled_tick == 115u && release.observed_tick == 119u && release.skipped == 2u);
    TEST_CHECK(PlatformPeriodicFinish(&clock, &release, 120u, &timing) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(!timing.over_budget && !timing.missed_deadline); // 紧贴下一拍时完成仍合格。
    TEST_CHECK(PlatformPeriodicFinish(&clock, &release, 121u, &timing) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(!timing.over_budget && timing.missed_deadline); // 工作本身只花 2 ms，唤醒过迟仍会错过截止点。
    TEST_CHECK(PlatformPeriodicFinish(&clock, &release, 118u, &timing) == PLATFORM_PERIODIC_CLOCK);
    TEST_CHECK(timing.work_ticks == 2u && timing.missed_deadline); // 无效读数保留旧结果。
    TEST_CHECK(PlatformPeriodicPoll(&clock, 118u, &release) == PLATFORM_PERIODIC_CLOCK);
    TEST_CHECK(clock.next_tick == 120u && clock.last_tick == 119u && release.observed_tick == 119u);
    TEST_CHECK(PlatformPeriodicPoll(&clock, 120u, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(release.scheduled_tick == 120u && release.skipped == 0u);
}

static void TestFreeRtosWait(void)
{
    PlatformPeriodicClock_t clock;
    PlatformPeriodicRelease_t release;
    const PlatformPeriodicConfig_t config = {.period_us = 5000u, .budget_us = 2000u}; // 固件 RTOS 分辨率为 1 ms。
    const PlatformPeriodicConfig_t unsupported = {.period_us = 500u, .budget_us = 100u}; // 软件刻度无法表示。
    now_tick = UINT32_MAX - 2u;
    TEST_CHECK(PlatformTaskPeriodicInit(&clock, &config) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(PlatformTaskPeriodicInit(&clock, &unsupported) == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(PlatformTaskPeriodicWait(&clock, &release) == PLATFORM_PERIODIC_READY && sleep_count == 0u);
    TEST_CHECK(release.scheduled_tick == UINT32_MAX - 2u);
    ++now_tick;
    wake_latency = 11u; // 预期在时刻 2 唤醒，实际到时刻 13 才继续执行。
    TEST_CHECK(PlatformTaskPeriodicWait(&clock, &release) == PLATFORM_PERIODIC_READY);
    TEST_CHECK(sleep_until == 2u && sleep_count == 1u && release.observed_tick == 13u);
    TEST_CHECK(release.scheduled_tick == 12u && release.skipped == 2u && clock.next_tick == 17u);
    wake_early = 1u;
    TEST_CHECK(PlatformTaskPeriodicWait(&clock, &release) == PLATFORM_PERIODIC_CLOCK);
    TEST_CHECK(clock.last_tick == 13u && clock.next_tick == 17u && release.observed_tick == 13u);
    scheduler_state = taskSCHEDULER_SUSPENDED;
    TEST_CHECK(PlatformTaskPeriodicWait(&clock, &release) == PLATFORM_PERIODIC_CONTEXT);
    scheduler_state = taskSCHEDULER_RUNNING;
    in_handler = 1u;
    TEST_CHECK(PlatformTaskPeriodicWait(&clock, &release) == PLATFORM_PERIODIC_CONTEXT && sleep_count == 2u);
}

int main(void)
{
    TestExactSetup();
    TestPlan();
    TestScheduleAndBudget();
    TestFreeRtosWait();
    return 0;
}
