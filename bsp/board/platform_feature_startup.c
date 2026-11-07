/** @file platform_feature_startup.c
 * @brief 将当前板载功能映射到 CubeMX 生成的 SPI 和 PWM 初始化入口。
 */
#include "platform_feature_startup.h"
#include "build_selection.h"
#include "spi.h"
#include "tim.h"

void PlatformFeaturePeripheralsInit(void)
{
#if MC02_FEATURE_ALARM
    MX_TIM12_Init(); // 板载蜂鸣器使用 TIM12；关闭告警时不配置该定时器。
#endif
#if MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS
    MX_SPI1_Init(); // 板载 LCD 使用 SPI1；LCD 诊断也需要该总线。
#endif
#if MC02_FEATURE_IMU
    MX_SPI2_Init(); // 当前板载 BMI088 使用 SPI2。
#endif
#if MC02_FEATURE_HEATER
    MX_TIM3_Init(); // 只有启用 IMU 温控时才配置加热 PWM。
#endif
}
