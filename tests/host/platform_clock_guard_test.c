/** @file platform_clock_guard_test.c
 * @brief 用可控计数检查 HAL 时钟配置的固定期限与停表兜底。
 */
#include "platform_clock_guard.h"
#include "test_check.h"
#include <limits.h>
#include <stdint.h>

int main(void)
{
    PlatformClockGuard_t guard = {7u, 6u, 5u, 4u, 3u, 1u}; // 保留初值以验证失败不覆盖。
    const PlatformClockGuard_t saved = guard; // 比较字段值，不依赖结构体填充字节。
    TEST_CHECK(!PlatformClockGuardBegin(NULL, 0u, 10u, 5u)); // 空输出不能开始。
    TEST_CHECK(!PlatformClockGuardBegin(&guard, 0u, 0u, 5u)); // 零周期上限无效。
    TEST_CHECK(!PlatformClockGuardBegin(&guard, 0u, (uint32_t)INT32_MAX + 1u, 5u)); // 回绕判断要求半周期以内。
    TEST_CHECK(!PlatformClockGuardBegin(&guard, 0u, 10u, 0u)); // 查询上限不能为零。
    TEST_CHECK(guard.started_cycles == saved.started_cycles && guard.cycle_limit == saved.cycle_limit &&
               guard.poll_limit == saved.poll_limit && guard.polls == saved.polls &&
               guard.forced_offset == saved.forced_offset && guard.expired == saved.expired); // 参数错误不改已有字段。
    TEST_CHECK(PlatformClockGuardBegin(&guard, UINT32_MAX - 2u, 8u, 10u)); // 从接近回绕的周期开始。
    TEST_CHECK(PlatformClockGuardTick(&guard, 30u, 1u) == 30u && !guard.expired); // 跨回绕仍在期限内。
    TEST_CHECK((uint32_t)(PlatformClockGuardTick(&guard, 30u, 5u) - 30u) > 5000u); // 达到固定期限后让 HAL 退出。
    TEST_CHECK(guard.expired); // 超时状态锁存。
    const uint32_t after_expiry = PlatformClockGuardTick(&guard, 30u, 6u); // 模拟 HAL 在超时后开始另一次等待。
    TEST_CHECK((uint32_t)(PlatformClockGuardTick(&guard, 30u, 6u) - after_expiry) > 5000u); // 新等待也必须退出。
    TEST_CHECK(PlatformClockGuardBegin(&guard, 10u, 100u, 3u)); // 重新开始另一轮配置。
    TEST_CHECK(PlatformClockGuardTick(&guard, 1u, 10u) == 1u); // 周期计数停止时仍可先检查。
    TEST_CHECK(PlatformClockGuardTick(&guard, 2u, 10u) == 2u); // HAL tick 变化不重置总查询次数。
    TEST_CHECK((uint32_t)(PlatformClockGuardTick(&guard, 3u, 10u) - 3u) > 5000u); // 次数用尽后不再无限等待。
    TEST_CHECK(guard.expired && guard.polls == 3u); // 保存真实触发原因及固定次数。
    return 0;
}
