/** @file feature_startup_test.c
 * @brief 使用生产板级入口检查启用功能对应的外设调用。
 */
#include "platform_feature_startup.h"
#include "test_check.h"

static unsigned spi1_calls, spi2_calls, tim12_calls, tim3_calls;
void MX_SPI1_Init(void) { ++spi1_calls; }
void MX_SPI2_Init(void) { ++spi2_calls; }
void MX_TIM12_Init(void) { ++tim12_calls; }
void MX_TIM3_Init(void) { ++tim3_calls; }

int main(void)
{
    PlatformFeaturePeripheralsInit(); // 使用当前测试组合执行真实选择入口。
    TEST_CHECK(spi1_calls == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(spi2_calls == MC02_FEATURE_IMU);
    TEST_CHECK(tim12_calls == MC02_FEATURE_ALARM);
    TEST_CHECK(tim3_calls == MC02_FEATURE_HEATER);
    return 0;
}
