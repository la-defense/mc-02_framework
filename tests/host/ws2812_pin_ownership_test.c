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
    (void)handle;
    (void)data;
    (void)size;
    (void)timeout;
    return HAL_OK;
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
    WS2812_Init();
    WS2812_Init();

    // 初始化可重复调用；PA5 仍由 ADC1 通道19读取，只有数据输出脚切到 SPI6。
    TEST_CHECK(clock_config_calls == 1u);
    TEST_CHECK(spi_init_calls == 1u);
    TEST_CHECK(gpio_init_calls == 1u);
    TEST_CHECK(configured_pins == GPIO_PIN_7);
    TEST_CHECK((configured_pins & GPIO_PIN_5) == 0u);
    TEST_CHECK(configured_mode == GPIO_MODE_AF_PP);
    TEST_CHECK(configured_alternate == GPIO_AF8_SPI6);

    return 0;
}
