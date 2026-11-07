/** @file platform_power_bench_test.c
 * @brief 验证裸板编译的电源后端不能开启任何外接供电。
 */
#include "platform_power.h"
#include "test_check.h"

int main(void)
{
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
        TEST_CHECK(!PlatformPowerEnable((PlatformPowerRail_t)rail)); // 裸板始终拒绝开启。
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
        TEST_CHECK(!PlatformPowerIsEnabled((PlatformPowerRail_t)rail)); // 裸板不报告开启。
    TEST_CHECK(!PlatformPowerEnable((PlatformPowerRail_t)PLATFORM_POWER_COUNT));
    return 0;
}
