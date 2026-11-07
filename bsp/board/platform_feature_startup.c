/** @file platform_feature_startup.c
 * @brief 将当前板载功能映射到 CubeMX 生成的 SPI 和 PWM 初始化入口。
 */
#include "platform_feature_startup.h"
#include "build_selection.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include <stddef.h>

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
