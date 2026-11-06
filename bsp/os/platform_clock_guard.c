/** @file platform_clock_guard.c
 * @brief 为启动时 HAL 时钟等待提供固定上限；不访问硬件或 RTOS。
 */
#include "platform_clock_guard.h"
#include <limits.h>
#include <stddef.h>

uint8_t PlatformClockGuardBegin(PlatformClockGuard_t *guard, uint32_t cycles,
                                uint32_t cycle_limit, uint32_t poll_limit)
{
    if (guard == NULL || cycle_limit == 0u || cycle_limit > INT32_MAX || poll_limit == 0u)
        return 0u; // 参数无效时不能覆盖上一次状态。
    const PlatformClockGuard_t candidate = {
        .started_cycles = cycles, // 固定本次开始时刻。
        .cycle_limit = cycle_limit, // 保存总周期上限。
        .poll_limit = poll_limit, // 保存总查询次数上限。
        .polls = 0u, // 新配置从零次查询开始。
        .forced_offset = 0u, // 正常状态不修改 HAL 时间。
        .expired = 0u, // 本次配置尚未超时。
    };
    *guard = candidate; // 所有参数通过后一次发布。
    return 1u; // 告诉调用者可以开始受保护的配置。
}

uint32_t PlatformClockGuardTick(PlatformClockGuard_t *guard, uint32_t raw_tick,
                                uint32_t cycles)
{
    if (guard == NULL) return raw_tick; // 空状态不改变调用者提供的时间。
    if (!guard->expired) // 超时一旦发生就不会被新的时间或调用清除。
    {
        ++guard->polls; // 计入本次查询；总次数不因时钟前进而重新计算。
        if ((uint32_t)(cycles - guard->started_cycles) >= guard->cycle_limit ||
            guard->polls >= guard->poll_limit)
            guard->expired = 1u; // 周期到期或计数停止时都能终止等待。
    }
    if (guard->expired)
        guard->forced_offset += CLOCK_GUARD_EXPIRED_STEP_MS; // 连续查询也能终止 HAL 新开始的等待。
    return raw_tick + guard->forced_offset; // 只返回临时值，不修改 HAL 全局计数。
}
