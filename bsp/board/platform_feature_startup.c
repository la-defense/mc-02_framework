/** @file platform_feature_startup.c
 * @brief 将当前板载功能映射到 CubeMX 生成的 SPI 和 PWM 初始化入口。
 */
#include "platform_feature_startup.h"
#include "build_selection.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "fdcan.h"
#include <stddef.h>

static uint8_t lcd_ready; // 只由启动阶段写入，供后续模块启动读取。
static uint8_t imu_ready; // 保存板载 IMU 的 SPI2 初始化结果。

void PlatformFeaturePeripheralsInit(void)
{
#if MC02_FEATURE_ALARM
    MX_TIM12_Init(); // 板载蜂鸣器使用 TIM12；关闭告警时不配置该定时器。
#endif
#if MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS
    lcd_ready = MX_SPI1_InitChecked(); // 失败时交给模块依赖处理，不启动 LCD。
#endif
#if MC02_FEATURE_IMU
    imu_ready = MX_SPI2_InitChecked(); // 失败时交给车型模块依赖处理。
#endif
#if MC02_FEATURE_HEATER
    MX_TIM3_Init(); // 只有启用 IMU 温控时才配置加热 PWM。
#endif
}

uint8_t PlatformFeatureLcdReady(void)
{
    return lcd_ready; // 未启用或初始化失败均保持不可用。
}

uint8_t PlatformFeatureImuReady(void)
{
    return imu_ready; // 未启用或初始化失败均不可注册板载 IMU。
}

uint8_t PlatformFeatureUartsInit(const PlatformUartPort_t *ports, uint8_t count)
{
    if (count > 3u || (count != 0u && ports == NULL)) return 0u; // 输入不完整时不操作任何串口。
    uint32_t selected = UINT32_C(1) << 10u; // BSP 日志和诊断固定使用 USART10。
    for (uint8_t index = 0u; index < count; ++index)
    {
        const unsigned port = (unsigned)ports[index]; // 转成无符号值，先判断再移位。
        switch (ports[index])
        {
        case PLATFORM_UART_1: case PLATFORM_UART_2: case PLATFORM_UART_3:
        case PLATFORM_UART_5: case PLATFORM_UART_7: case PLATFORM_UART_8:
        case PLATFORM_UART_9: case PLATFORM_UART_10:
            selected |= UINT32_C(1) << port; // 同一串口多次声明仍只设置一位。
            break;
        default:
            return 0u; // 无板级绑定的编号不启动任何串口。
        }
    }
    if ((selected & (UINT32_C(1) << 1u)) != 0u) MX_USART1_UART_Init(); // 逻辑串口 1。
    if ((selected & (UINT32_C(1) << 2u)) != 0u) MX_USART2_UART_Init(); // 逻辑串口 2。
    if ((selected & (UINT32_C(1) << 3u)) != 0u) MX_USART3_UART_Init(); // 逻辑串口 3。
    if ((selected & (UINT32_C(1) << 5u)) != 0u) MX_UART5_Init(); // 逻辑串口 5。
    if ((selected & (UINT32_C(1) << 7u)) != 0u) MX_UART7_Init(); // 逻辑串口 7。
    if ((selected & (UINT32_C(1) << 8u)) != 0u) MX_UART8_Init(); // 逻辑串口 8。
    if ((selected & (UINT32_C(1) << 9u)) != 0u) MX_UART9_Init(); // 逻辑串口 9。
    MX_USART10_UART_Init(); // 固定诊断串口已包含在集合中，始终只初始化一次。
    return 1u; // 所有需要的初始化入口都已调用。
}

uint8_t PlatformFeatureCanInit(const PlatformCanBus_t *buses, uint8_t count)
{
    if (count > 8u || (count != 0u && buses == NULL)) return 0u; // 先检查容量和数组。
    uint32_t selected = 0u; // 未请求总线时不初始化 CAN 控制器。
    for (uint8_t index = 0u; index < count; ++index)
    {
        const unsigned bus = (unsigned)buses[index]; // 在移位前检查编号范围。
        if (bus < PLATFORM_CAN_BUS_1 || bus > PLATFORM_CAN_BUS_3)
            return 0u; // 存在未知总线时不进行任何硬件初始化。
        selected |= UINT32_C(1) << bus; // 不同设备可以共同使用一条总线。
    }
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_1)) != 0u) MX_FDCAN1_Init(); // 板级总线 1。
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_2)) != 0u) MX_FDCAN2_Init(); // 板级总线 2。
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_3)) != 0u) MX_FDCAN3_Init(); // 板级总线 3。
    return 1u; // 已调用所有需要的控制器初始化入口。
}
