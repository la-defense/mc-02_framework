/** @file feature_startup_test.c
 * @brief 使用生产板级入口检查启用功能对应的外设调用。
 */
#include "platform_feature_startup.h"
#include "test_check.h"
#include <stddef.h>

static unsigned spi1_calls, spi2_calls, tim12_calls, tim3_calls;
static unsigned uart_calls[11];
static unsigned can_calls[4];
static unsigned char fake_spi1_success = 1u;
void MX_SPI1_Init(void) { ++spi1_calls; }
unsigned char MX_SPI1_InitChecked(void) { ++spi1_calls; return fake_spi1_success; }
void MX_SPI2_Init(void) { ++spi2_calls; }
void MX_TIM12_Init(void) { ++tim12_calls; }
void MX_TIM3_Init(void) { ++tim3_calls; }
void MX_USART1_UART_Init(void) { ++uart_calls[1]; }
void MX_USART2_UART_Init(void) { ++uart_calls[2]; }
void MX_USART3_UART_Init(void) { ++uart_calls[3]; }
void MX_UART5_Init(void) { ++uart_calls[5]; }
void MX_UART7_Init(void) { ++uart_calls[7]; }
void MX_UART8_Init(void) { ++uart_calls[8]; }
void MX_UART9_Init(void) { ++uart_calls[9]; }
void MX_USART10_UART_Init(void) { ++uart_calls[10]; }
void MX_FDCAN1_Init(void) { ++can_calls[1]; }
void MX_FDCAN2_Init(void) { ++can_calls[2]; }
void MX_FDCAN3_Init(void) { ++can_calls[3]; }

int main(void)
{
    PlatformFeaturePeripheralsInit(); // 使用当前测试组合执行真实选择入口。
    TEST_CHECK(spi1_calls == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(PlatformFeatureLcdReady() == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(spi2_calls == MC02_FEATURE_IMU);
    TEST_CHECK(tim12_calls == MC02_FEATURE_ALARM);
    TEST_CHECK(tim3_calls == MC02_FEATURE_HEATER);
#if MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS
    fake_spi1_success = 0u; // 模拟 LCD 的 SPI1 初始化失败。
    PlatformFeaturePeripheralsInit();
    TEST_CHECK(!PlatformFeatureLcdReady()); // 模块启动不得报告 LCD 已可用。
#endif
    const PlatformUartPort_t invalid[] = {PLATFORM_UART_1, PLATFORM_UART_INVALID};
    TEST_CHECK(!PlatformFeatureUartsInit(invalid, 2u)); // 非法编号不能产生部分硬件初始化。
    TEST_CHECK(uart_calls[1] == 0u && uart_calls[10] == 0u);
    TEST_CHECK(!PlatformFeatureUartsInit(NULL, 1u)); // 数组缺失时立即拒绝。
    TEST_CHECK(!PlatformFeatureUartsInit(invalid, 4u)); // 超过固定容量时不读取数组。
    const PlatformUartPort_t ports[] = {PLATFORM_UART_5, PLATFORM_UART_5, PLATFORM_UART_10};
    TEST_CHECK(PlatformFeatureUartsInit(ports, 3u)); // 重复声明只启动一次。
    TEST_CHECK(uart_calls[5] == 1u && uart_calls[10] == 1u);
    TEST_CHECK(uart_calls[1] == 0u && uart_calls[2] == 0u && uart_calls[3] == 0u);
    TEST_CHECK(uart_calls[7] == 0u && uart_calls[8] == 0u && uart_calls[9] == 0u);
    const PlatformUartPort_t first[] = {PLATFORM_UART_1, PLATFORM_UART_2, PLATFORM_UART_3};
    const PlatformUartPort_t second[] = {PLATFORM_UART_7, PLATFORM_UART_8, PLATFORM_UART_9};
    TEST_CHECK(PlatformFeatureUartsInit(first, 3u)); // 覆盖板级表中的其他编号。
    TEST_CHECK(PlatformFeatureUartsInit(second, 3u));
    TEST_CHECK(uart_calls[1] == 1u && uart_calls[2] == 1u && uart_calls[3] == 1u);
    TEST_CHECK(uart_calls[7] == 1u && uart_calls[8] == 1u && uart_calls[9] == 1u);
    TEST_CHECK(uart_calls[10] == 3u); // 每次调用均保留一次诊断初始化。
    const PlatformCanBus_t invalid_can[] = {PLATFORM_CAN_BUS_1, PLATFORM_CAN_INVALID};
    TEST_CHECK(!PlatformFeatureCanInit(invalid_can, 2u)); // 无效总线不能导致部分初始化。
    TEST_CHECK(!PlatformFeatureCanInit(NULL, 1u));
    TEST_CHECK(!PlatformFeatureCanInit(invalid_can, 9u));
    TEST_CHECK(can_calls[1] == 0u && can_calls[2] == 0u && can_calls[3] == 0u);
    TEST_CHECK(PlatformFeatureCanInit(NULL, 0u)); // 没有 CAN 设备时保持全部控制器未初始化。
    TEST_CHECK(can_calls[1] == 0u && can_calls[2] == 0u && can_calls[3] == 0u);
    const PlatformCanBus_t buses[] = {PLATFORM_CAN_BUS_2, PLATFORM_CAN_BUS_2, PLATFORM_CAN_BUS_3};
    TEST_CHECK(PlatformFeatureCanInit(buses, 3u)); // 重复请求的控制器只配置一次。
    TEST_CHECK(can_calls[1] == 0u && can_calls[2] == 1u && can_calls[3] == 1u);
    const PlatformCanBus_t first_bus[] = {PLATFORM_CAN_BUS_1};
    TEST_CHECK(PlatformFeatureCanInit(first_bus, 1u));
    TEST_CHECK(can_calls[1] == 1u && can_calls[2] == 1u && can_calls[3] == 1u);
    return 0;
}
