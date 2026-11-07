/** @file feature_startup_test.c
 * @brief 使用生产板级入口检查启用功能对应的外设调用。
 */
#include "platform_feature_startup.h"
#include "test_check.h"
#include <stddef.h>

static unsigned spi1_calls, spi2_calls, tim12_calls, tim3_calls;
static unsigned uart_calls[11];
void MX_SPI1_Init(void) { ++spi1_calls; }
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

int main(void)
{
    PlatformFeaturePeripheralsInit(); // 使用当前测试组合执行真实选择入口。
    TEST_CHECK(spi1_calls == (MC02_FEATURE_LCD || MC02_BENCH_LCD_DIAGNOSTICS));
    TEST_CHECK(spi2_calls == MC02_FEATURE_IMU);
    TEST_CHECK(tim12_calls == MC02_FEATURE_ALARM);
    TEST_CHECK(tim3_calls == MC02_FEATURE_HEATER);
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
    return 0;
}
