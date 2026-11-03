/** @file platform_timeline_ms_test.c
 * @brief 检查浮点毫秒诊断时间由平台后端原样交付。
 */
#include "test_check.h"
#include "platform_probe.h"

float DWT_GetTimeline_ms(void) { return 1234.5f; }

int main(void)
{
    TEST_CHECK(PlatformProbeTimelineMs() == 1234.5f);
    return 0;
}
