/** @file platform_feature_startup.c
 * @brief 将当前板载功能映射到 CubeMX 生成的 SPI 和 PWM 初始化入口。
 */
#include "platform_feature_startup.h"
#include "build_selection.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "fdcan.h"
#include "adc.h"
#include <stddef.h>

static uint8_t lcd_ready; // 只由启动阶段写入，供后续模块启动读取。
static uint8_t imu_ready; // 保存板载 IMU 的 SPI2 初始化结果。
static uint8_t heater_ready; // 保存 IMU 加热定时器初始化结果。
static uint8_t can_ready; // 每位记录一条已完成板级初始化的 CAN 总线。
static uint32_t uart_ready; // 每位记录对应逻辑串口是否完成初始化。
static uint8_t adc_ready; // 板载 VIN 和 LCD 按键共用 ADC1。

void PlatformFeaturePeripheralsInit(void)
{
#if MC02_FEATURE_ADC
    adc_ready = MX_ADC1_InitChecked(); // 当前板有使用者时准备 ADC。
#endif
#if MC02_FEATURE_ALARM
    (void)MX_TIM12_InitChecked(); // 失败时句柄无效，告警模块仍可保留 RGB。
#endif
#if MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS
    lcd_ready = MX_SPI1_InitChecked(); // 失败时交给模块依赖处理，不启动 LCD。
#endif
#if MC02_FEATURE_IMU
    imu_ready = MX_SPI2_InitChecked(); // 失败时交给车型模块依赖处理。
#endif
#if MC02_FEATURE_HEATER
    heater_ready = MX_TIM3_InitChecked(); // 失败时由车型依赖决定是否允许其他模块运行。
#endif
}

uint8_t PlatformFeatureAdcSelected(void)
{
    return MC02_FEATURE_ADC; // 供 BSP 区分未选择与初始化失败。
}

uint8_t PlatformFeatureAdcReady(void)
{
    return adc_ready; // 未准备或失败时不能进入采样启动。
}

uint8_t PlatformFeatureLcdReady(void)
{
    return lcd_ready; // 未启用或初始化失败均保持不可用。
}

uint8_t PlatformFeatureImuReady(void)
{
    return imu_ready; // 未启用或初始化失败均不可注册板载 IMU。
}

uint8_t PlatformFeatureHeaterReady(void)
{
    return heater_ready; // 未启用或初始化失败时不能启动温控。
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
    if ((selected & (UINT32_C(1) << 1u)) && MX_USART1_UART_InitChecked()) uart_ready |= UINT32_C(1) << 1u; // USART1。
    if ((selected & (UINT32_C(1) << 2u)) && MX_USART2_UART_InitChecked()) uart_ready |= UINT32_C(1) << 2u; // USART2。
    if ((selected & (UINT32_C(1) << 3u)) && MX_USART3_UART_InitChecked()) uart_ready |= UINT32_C(1) << 3u; // USART3。
    if ((selected & (UINT32_C(1) << 5u)) && MX_UART5_InitChecked()) uart_ready |= UINT32_C(1) << 5u; // UART5。
    if ((selected & (UINT32_C(1) << 7u)) && MX_UART7_InitChecked()) uart_ready |= UINT32_C(1) << 7u; // UART7。
    if ((selected & (UINT32_C(1) << 8u)) && MX_UART8_InitChecked()) uart_ready |= UINT32_C(1) << 8u; // UART8。
    if ((selected & (UINT32_C(1) << 9u)) && MX_UART9_InitChecked()) uart_ready |= UINT32_C(1) << 9u; // UART9。
    if (MX_USART10_UART_InitChecked()) uart_ready |= UINT32_C(1) << 10u; // 诊断串口仅初始化一次。
    return 1u; // 编号有效；各端口失败由依赖它的模块处理。
}

uint8_t PlatformFeatureUartReady(PlatformUartPort_t port)
{
    const unsigned value = (unsigned)port; // 检查范围后才能移位。
    if (value > 10u) return 0u; // 未绑定或非法编号不可能就绪。
    return (uint8_t)((uart_ready & (UINT32_C(1) << value)) != 0u); // 未选择或失败均不可用。
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
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_1)) != 0u && MX_FDCAN1_InitChecked())
        can_ready |= (uint8_t)(1u << PLATFORM_CAN_BUS_1); // 保存总线 1 的实际结果。
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_2)) != 0u && MX_FDCAN2_InitChecked())
        can_ready |= (uint8_t)(1u << PLATFORM_CAN_BUS_2); // 单条失败不妨碍其他总线。
    if ((selected & (UINT32_C(1) << PLATFORM_CAN_BUS_3)) != 0u && MX_FDCAN3_InitChecked())
        can_ready |= (uint8_t)(1u << PLATFORM_CAN_BUS_3); // 记录总线 3 的结果。
    return 1u; // 编号有效；各控制器失败由使用该总线的模块处理。
}

uint8_t PlatformFeatureCanReady(PlatformCanBus_t bus)
{
    if (bus < PLATFORM_CAN_BUS_1 || bus > PLATFORM_CAN_BUS_3) return 0u; // 越界编号没有可用控制器。
    return (uint8_t)((can_ready & (1u << (unsigned)bus)) != 0u); // 未选择或准备失败均不可用。
}
