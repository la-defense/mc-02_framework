/** @file feature_startup_test.c
 * @brief 使用生产板级入口检查启用功能对应的外设调用。
 */
#include "platform_feature_startup.h"
#include "test_check.h"
#include <stddef.h>

static unsigned spi1_calls, spi2_calls, tim12_calls, tim3_calls;
static unsigned uart_calls[11];
static unsigned failed_uart; // 选定一个端口模拟硬件失败。
static unsigned adc_calls;
static unsigned char fake_adc_success = 1u;
static unsigned can_calls[4];
static unsigned failed_can;
static unsigned char fake_spi1_success = 1u;
static unsigned char fake_spi2_success = 1u;
static unsigned char fake_tim3_success = 1u;
void MX_SPI1_Init(void) { ++spi1_calls; }
unsigned char MX_ADC1_InitChecked(void) { ++adc_calls; return fake_adc_success; }
unsigned char MX_SPI1_InitChecked(void) { ++spi1_calls; return fake_spi1_success; }
void MX_SPI2_Init(void) { ++spi2_calls; }
unsigned char MX_SPI2_InitChecked(void) { ++spi2_calls; return fake_spi2_success; }
void MX_TIM12_Init(void) { ++tim12_calls; }
void MX_TIM3_Init(void) { ++tim3_calls; }
unsigned char MX_TIM12_InitChecked(void) { ++tim12_calls; return 1u; }
unsigned char MX_TIM3_InitChecked(void) { ++tim3_calls; return fake_tim3_success; }
void MX_USART1_UART_Init(void) { ++uart_calls[1]; }
void MX_USART2_UART_Init(void) { ++uart_calls[2]; }
void MX_USART3_UART_Init(void) { ++uart_calls[3]; }
void MX_UART5_Init(void) { ++uart_calls[5]; }
void MX_UART7_Init(void) { ++uart_calls[7]; }
void MX_UART8_Init(void) { ++uart_calls[8]; }
void MX_UART9_Init(void) { ++uart_calls[9]; }
void MX_USART10_UART_Init(void) { ++uart_calls[10]; }
unsigned char MX_USART1_UART_InitChecked(void) { ++uart_calls[1]; return failed_uart != 1u; }
unsigned char MX_USART2_UART_InitChecked(void) { ++uart_calls[2]; return failed_uart != 2u; }
unsigned char MX_USART3_UART_InitChecked(void) { ++uart_calls[3]; return failed_uart != 3u; }
unsigned char MX_UART5_InitChecked(void) { ++uart_calls[5]; return failed_uart != 5u; }
unsigned char MX_UART7_InitChecked(void) { ++uart_calls[7]; return failed_uart != 7u; }
unsigned char MX_UART8_InitChecked(void) { ++uart_calls[8]; return failed_uart != 8u; }
unsigned char MX_UART9_InitChecked(void) { ++uart_calls[9]; return failed_uart != 9u; }
unsigned char MX_USART10_UART_InitChecked(void) { ++uart_calls[10]; return failed_uart != 10u; }
void MX_FDCAN1_Init(void) { ++can_calls[1]; }
void MX_FDCAN2_Init(void) { ++can_calls[2]; }
void MX_FDCAN3_Init(void) { ++can_calls[3]; }
unsigned char MX_FDCAN1_InitChecked(void) { ++can_calls[1]; return failed_can != 1u; }
unsigned char MX_FDCAN2_InitChecked(void) { ++can_calls[2]; return failed_can != 2u; }
unsigned char MX_FDCAN3_InitChecked(void) { ++can_calls[3]; return failed_can != 3u; }

int main(void)
{
    PlatformFeaturePeripheralsInit(); // 使用当前测试组合执行真实选择入口。
    TEST_CHECK(adc_calls == MC02_FEATURE_ADC && PlatformFeatureAdcSelected() == MC02_FEATURE_ADC);
    TEST_CHECK(PlatformFeatureAdcReady() == MC02_FEATURE_ADC); // 未选择时不访问 ADC。
    TEST_CHECK(spi1_calls == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(PlatformFeatureLcdReady() == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(spi2_calls == MC02_FEATURE_IMU);
    TEST_CHECK(PlatformFeatureImuReady() == MC02_FEATURE_IMU);
    TEST_CHECK(tim12_calls == MC02_FEATURE_ALARM);
    TEST_CHECK(tim3_calls == MC02_FEATURE_HEATER);
    TEST_CHECK(PlatformFeatureHeaterReady() == MC02_FEATURE_HEATER);
#if MC02_FEATURE_ADC
    fake_adc_success = 0u; // 板级 ADC 失败不影响其他板级入口。
    PlatformFeaturePeripheralsInit();
    TEST_CHECK(!PlatformFeatureAdcReady());
#endif
#if MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS
    fake_spi1_success = 0u; // 模拟 LCD 的 SPI1 初始化失败。
    PlatformFeaturePeripheralsInit();
    TEST_CHECK(!PlatformFeatureLcdReady()); // 模块启动不得报告 LCD 已可用。
#endif
#if MC02_FEATURE_HEATER
    fake_tim3_success = 0u; // 模拟加热 PWM 初始化失败。
    PlatformFeaturePeripheralsInit();
    TEST_CHECK(!PlatformFeatureHeaterReady()); // 温控不能继续使用失败的定时器。
#endif
#if MC02_FEATURE_IMU
    fake_spi2_success = 0u; // 模拟 IMU 所需 SPI2 初始化失败。
    PlatformFeaturePeripheralsInit();
    TEST_CHECK(!PlatformFeatureImuReady()); // 不能注册依赖无效 SPI 的 IMU。
#endif
    const PlatformUartPort_t invalid[] = {PLATFORM_UART_1, PLATFORM_UART_INVALID};
    TEST_CHECK(!PlatformFeatureUartsInit(invalid, 2u)); // 非法编号不能产生部分硬件初始化。
    TEST_CHECK(uart_calls[1] == 0u && uart_calls[10] == 0u);
    TEST_CHECK(!PlatformFeatureUartsInit(NULL, 1u)); // 数组缺失时立即拒绝。
    TEST_CHECK(!PlatformFeatureUartsInit(invalid, 4u)); // 超过固定容量时不读取数组。
    const PlatformUartPort_t ports[] = {PLATFORM_UART_5, PLATFORM_UART_5, PLATFORM_UART_10};
    failed_uart = 5u; // 一路业务串口失败时仍准备诊断串口。
    TEST_CHECK(PlatformFeatureUartsInit(ports, 3u)); // 重复声明只启动一次。
    TEST_CHECK(uart_calls[5] == 1u && uart_calls[10] == 1u);
    TEST_CHECK(!PlatformFeatureUartReady(PLATFORM_UART_5)); // 失败端口不能供模块注册。
    TEST_CHECK(PlatformFeatureUartReady(PLATFORM_UART_10)); // 其他端口保持可用。
    TEST_CHECK(!PlatformFeatureUartReady(PLATFORM_UART_INVALID)); // 未绑定编号不能就绪。
    TEST_CHECK(uart_calls[1] == 0u && uart_calls[2] == 0u && uart_calls[3] == 0u);
    TEST_CHECK(uart_calls[7] == 0u && uart_calls[8] == 0u && uart_calls[9] == 0u);
    const PlatformUartPort_t first[] = {PLATFORM_UART_1, PLATFORM_UART_2, PLATFORM_UART_3};
    const PlatformUartPort_t second[] = {PLATFORM_UART_7, PLATFORM_UART_8, PLATFORM_UART_9};
    failed_uart = 0u; // 后续选择其他端口应继续成功。
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
    failed_can = 3u; // 第三条初始化失败不能阻止第二条准备。
    TEST_CHECK(PlatformFeatureCanInit(buses, 3u)); // 重复请求的控制器只配置一次。
    TEST_CHECK(can_calls[1] == 0u && can_calls[2] == 1u && can_calls[3] == 1u);
    TEST_CHECK(PlatformFeatureCanReady(PLATFORM_CAN_BUS_2));
    TEST_CHECK(!PlatformFeatureCanReady(PLATFORM_CAN_BUS_3));
    const PlatformCanBus_t first_bus[] = {PLATFORM_CAN_BUS_1};
    failed_can = 0u;
    TEST_CHECK(PlatformFeatureCanInit(first_bus, 1u));
    TEST_CHECK(can_calls[1] == 1u && can_calls[2] == 1u && can_calls[3] == 1u);
    TEST_CHECK(PlatformFeatureCanReady(PLATFORM_CAN_BUS_1));
    return 0;
}
