/** @file platform_can_diagnostic_test.c
 * @brief 使用替身验证显式启用时回环诊断的调用及结果传递。
 */
#include "test_check.h"
#include "platform_can_diagnostic.h"
#include "platform_feature_startup.h"

static unsigned calls;
static uint8_t result;
static uint8_t ready_mask;
uint8_t CANRunLoopbackSelfTest(void) { ++calls; return result; }
uint8_t PlatformFeatureCanReady(PlatformCanBus_t bus)
{ return (uint8_t)((ready_mask & (1u << (unsigned)bus)) != 0u); }

int main(void)
{
    ready_mask = (1u << PLATFORM_CAN_BUS_1) | (1u << PLATFORM_CAN_BUS_2); // 第三条未准备时不能访问回环。
    TEST_CHECK(!PlatformCanRunLoopbackDiagnostic() && calls == 0u);
    ready_mask |= 1u << PLATFORM_CAN_BUS_3; // 三条总线准备后才允许执行回环。
    result = 0u;
    TEST_CHECK(PlatformCanRunLoopbackDiagnostic() == 0u && calls == 1u);
    result = 1u;
    TEST_CHECK(PlatformCanRunLoopbackDiagnostic() == 1u && calls == 2u);
    return 0;
}
