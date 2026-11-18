/** @file platform_periodic.c
 * @brief 计算固定拍点、跨回绕跳拍和完成时限，无硬件或 RTOS 依赖。
 */
#include "platform_periodic.h"
#include <limits.h>
#include <stddef.h>

PlatformPeriodicResult_t PlatformPeriodicPlan(const PlatformPeriodicRequest_t *requests, size_t count,
    const PlatformPeriodicCapabilities_t *capabilities, uint32_t *hardware_mask, size_t *failed_index)
{
    if ((requests == NULL && count != 0u) || count > 32u || capabilities == NULL || hardware_mask == NULL)
        return PLATFORM_PERIODIC_CONFIG; // 参数无效时保留两个输出。
    uint32_t remaining = capabilities->high_res_channels; // 各任务需要独立比较通道。
    uint32_t staged_mask = 0u; // 所有任务通过后才向调用者交付选择。
    for (size_t i = 0; i < count; ++i) // 按配置顺序检查实际启用的任务。
    {
        const PlatformPeriodicRequest_t *request = &requests[i]; // 读取本任务声明的时间和允许来源。
        PlatformPeriodicClock_t example; // 复用正式等待接口使用的精度、预算和回绕检查。
        if ((request->source == PLATFORM_PERIODIC_SOURCE_AUTO || request->source == PLATFORM_PERIODIC_SOURCE_RTOS) &&
            PlatformPeriodicInit(&example, &request->timing, capabilities->rtos_quantum_us, 0u) ==
            PLATFORM_PERIODIC_READY) continue; // RTOS 能精确表示时不占硬件通道。
        PlatformPeriodicResult_t result = PLATFORM_PERIODIC_CONFIG; // 不支持的来源或刻度均按配置失败。
        if (request->source == PLATFORM_PERIODIC_SOURCE_AUTO || request->source == PLATFORM_PERIODIC_SOURCE_HIGH_RES)
        {
            result = PlatformPeriodicInit(&example, &request->timing, capabilities->high_res_quantum_us, 0u);
            if (result == PLATFORM_PERIODIC_READY && remaining == 0u)
                result = PLATFORM_PERIODIC_RESOURCE; // 数值有效但已无独立比较通道。
        }
        if (result != PLATFORM_PERIODIC_READY) // 任一任务不可用则整组选择无效。
        {
            if (failed_index != NULL) *failed_index = i; // 将具体错项留给启动诊断。
            return result; // 调用者原有位图保持不变。
        }
        --remaining; // 给本任务保留一个计划通道。
        staged_mask |= UINT32_C(1) << i; // 对应位标记硬件来源。
    }
    *hardware_mask = staged_mask; // 一次交付完整选择。
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config, uint32_t quantum_us, uint32_t start_tick)
{
    if (clock == NULL || config == NULL || quantum_us == 0u || config->budget_us == 0u ||
        config->period_us < config->budget_us || config->period_us % quantum_us != 0u ||
        config->budget_us % quantum_us != 0u)
        return PLATFORM_PERIODIC_CONFIG; // 无法准确表示时直接拒绝，不改变调用者已有配置。
    const uint32_t period = config->period_us / quantum_us; // 将微秒换成后端计数刻度。
    if (period > INT32_MAX) return PLATFORM_PERIODIC_CONFIG; // 半个计数范围内才可判断回绕前后。
    *clock = (PlatformPeriodicClock_t){
        .period_ticks = period, // 周期直到重新初始化才改变。
        .budget_ticks = config->budget_us / quantum_us, // 保留与周期相同的计数单位。
        .next_tick = start_tick, // 第一拍在调用者指定的时刻可运行。
        .last_tick = start_tick, // 此前的时刻不属于本次调度。
    };
    return PLATFORM_PERIODIC_READY;
}

static uint8_t ClockValid(const PlatformPeriodicClock_t *clock)
{
    return (uint8_t)(clock != NULL && clock->budget_ticks != 0u &&
        clock->period_ticks >= clock->budget_ticks && clock->period_ticks <= INT32_MAX);
}

PlatformPeriodicResult_t PlatformPeriodicPoll(PlatformPeriodicClock_t *clock, uint32_t now_tick,
    PlatformPeriodicRelease_t *release)
{
    if (!ClockValid(clock) || release == NULL) return PLATFORM_PERIODIC_CONFIG; // 缺少初始化状态时禁止运行。
    const uint32_t advance = now_tick - clock->last_tick; // 正常回绕的前进距离仍可用无符号减法计算。
    if (advance > INT32_MAX) return PLATFORM_PERIODIC_CLOCK; // 拒绝倒退或间隔不明的计数。
    const uint32_t elapsed = now_tick - clock->next_tick; // 与最近未领取的计划拍比较。
    PlatformPeriodicRelease_t result = {.scheduled_tick = clock->next_tick, .observed_tick = now_tick}; // 等待和就绪共用同一输出。
    clock->last_tick = now_tick; // 此次观察本身也提供下一轮的时钟前进基准。
    if (elapsed > INT32_MAX)
    {
        result.wait_ticks = clock->next_tick - now_tick; // 下一拍还在将来，计算应让出的刻度数。
        *release = result; // 等待时不会移动计划拍点。
        return PLATFORM_PERIODIC_WAIT;
    }
    result.skipped = elapsed / clock->period_ticks; // 完整过去的每个周期只计数，不补跑。
    result.scheduled_tick += result.skipped * clock->period_ticks; // 领取最近一个到期的拍点。
    clock->next_tick = result.scheduled_tick + clock->period_ticks; // 继续沿原固定相位推进。
    *release = result; // 向任务交付本次运行时刻和跳拍信息。
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformPeriodicFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, uint32_t finish_tick, PlatformPeriodicTiming_t *timing)
{
    if (!ClockValid(clock) || release == NULL || timing == NULL || release->wait_ticks != 0u ||
        clock->next_tick != release->scheduled_tick + clock->period_ticks ||
        (uint32_t)(release->observed_tick - release->scheduled_tick) >= clock->period_ticks)
        return PLATFORM_PERIODIC_CONFIG; // 只接收当前已经领取的一个拍点。
    const uint32_t work = finish_tick - release->observed_tick; // 计算任务本轮实际经过的时间。
    const uint32_t from_schedule = finish_tick - release->scheduled_tick; // 包括开始前的调度延迟。
    if (work > INT32_MAX || from_schedule > INT32_MAX) return PLATFORM_PERIODIC_CLOCK; // 拒绝时钟倒退或不明确的跨圈间隔。
    *timing = (PlatformPeriodicTiming_t){
        .work_ticks = work, // 交付已测时间，单位仍为后端刻度。
        .over_budget = (uint8_t)(work > clock->budget_ticks), // 正好用完预算仍合格。
        .missed_deadline = (uint8_t)(from_schedule > clock->period_ticks), // 独立记录未能在下一计划拍前完成。
    };
    return PLATFORM_PERIODIC_READY;
}
