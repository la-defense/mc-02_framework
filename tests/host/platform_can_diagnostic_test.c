/** @file platform_can_diagnostic_test.c
 * @brief 使用替身验证显式启用时回环诊断的调用及结果传递。
 */
#include "test_check.h"
#include "platform_can_diagnostic.h"

static unsigned calls;
static uint8_t result;
uint8_t CANRunLoopbackSelfTest(void) { ++calls; return result; }

int main(void)
{
    result = 0u;
    TEST_CHECK(PlatformCanRunLoopbackDiagnostic() == 0u && calls == 1u);
    result = 1u;
    TEST_CHECK(PlatformCanRunLoopbackDiagnostic() == 1u && calls == 2u);
    return 0;
}
