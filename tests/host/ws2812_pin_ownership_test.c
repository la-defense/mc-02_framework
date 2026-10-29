/**
 * @file ws2812_pin_ownership_test.c
 * @brief 检查生产 WS2812 初始化不会把 ADC 按键使用的 PA5 改成 SPI 复用。
 * @note 测试替换 HAL 调用，只能验证软件请求的 GPIO 配置，不能证明板上 LED 波形。
 */
#include "test_check.h"
#include "ws2812.h"

#include <stddef.h>

GPIO_TypeDef fake_gpioa;

static uint32_t configured_pins;
static uint32_t configured_mode;
static uint32_t configured_alternate;
static unsigned gpio_init_calls;
static unsigned spi_init_calls;
static unsigned clock_config_calls;
static unsigned spi_transmit_calls;
static uint16_t spi_transmit_size;
static uint32_t spi_transmit_timeout;
static uint8_t spi_transmit_data[124];
static HAL_StatusTypeDef spi_transmit_result = HAL_OK;
static uint8_t tick_available = 1u;
static unsigned tick_acquires;
static unsigned tick_releases;

uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    if (tick_available == 0u)
        return 0u;
    ++tick_acquires;
    return 1u;
}

void BSP_HALTickReleaseBlockingWait(void)
{
    ++tick_releases;
}

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config)
{
    TEST_CHECK(config != NULL);
    TEST_CHECK(config->PeriphClockSelection == RCC_PERIPHCLK_SPI6);
    TEST_CHECK(config->Spi6ClockSelection == RCC_SPI6CLKSOURCE_HSE);
    ++clock_config_calls;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle)
{
    TEST_CHECK(handle != NULL);
    TEST_CHECK(handle->Instance == SPI6);
    handle->State = HAL_SPI_STATE_READY;
    ++spi_init_calls;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data,
                                   uint16_t size, uint32_t timeout)
{
    TEST_CHECK(handle != NULL && handle->Instance == SPI6);
    TEST_CHECK(data != NULL && size == sizeof(spi_transmit_data));
    ++spi_transmit_calls;
    spi_transmit_size = size;
    spi_transmit_timeout = timeout;
    for (uint16_t i = 0u; i < size; ++i)
        spi_transmit_data[i] = data[i];
    return spi_transmit_result;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    TEST_CHECK(port == GPIOA);
    TEST_CHECK(config != NULL);
    configured_pins |= config->Pin;
    configured_mode = config->Mode;
    configured_alternate = config->Alternate;
    ++gpio_init_calls;
}

void Error_Handler(void)
{
    abort();
}

int main(void)
{
    TEST_CHECK(WS2812_Init() == HAL_OK);
    TEST_CHECK(WS2812_Init() == HAL_OK);

    // 初始化可重复调用；PA5 仍由 ADC1 通道19读取，只有数据输出脚切到 SPI6。
    TEST_CHECK(clock_config_calls == 1u);
    TEST_CHECK(spi_init_calls == 1u);
    TEST_CHECK(gpio_init_calls == 1u);
    TEST_CHECK(configured_pins == GPIO_PIN_7);
    TEST_CHECK((configured_pins & GPIO_PIN_5) == 0u);
    TEST_CHECK(configured_mode == GPIO_MODE_AF_PP);
    TEST_CHECK(configured_alternate == GPIO_AF8_SPI6);

    TEST_CHECK(WS2812_SetRGB(0x80u, 0x00u, 0x01u) == HAL_OK);
    TEST_CHECK(spi_transmit_calls == 1u && spi_transmit_size == 124u);
    TEST_CHECK(spi_transmit_timeout == 2u);
    for (uint16_t i = 0u; i < 8u; ++i)
        TEST_CHECK(spi_transmit_data[i] == 0x60u);
    TEST_CHECK(spi_transmit_data[8] == 0x78u);
    for (uint16_t i = 9u; i < 23u; ++i)
        TEST_CHECK(spi_transmit_data[i] == 0x60u);
    TEST_CHECK(spi_transmit_data[23] == 0x78u);
    for (uint16_t i = 24u; i < 124u; ++i)
        TEST_CHECK(spi_transmit_data[i] == 0u);

    spi_transmit_result = HAL_BUSY;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    spi_transmit_result = HAL_ERROR;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_ERROR);
    TEST_CHECK(spi_transmit_calls == 3u);
    TEST_CHECK(tick_acquires == 3u && tick_releases == 3u);

    // 时基暂停时不能调用依赖 HAL 毫秒超时的阻塞发送。
    tick_available = 0u;
    TEST_CHECK(WS2812_SetRGB(1u, 2u, 3u) == HAL_BUSY);
    TEST_CHECK(spi_transmit_calls == 3u);
    TEST_CHECK(tick_acquires == 3u && tick_releases == 3u);

    return 0;
}
