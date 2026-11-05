/** @file power_gpio_test.c
 * @brief 用 HAL 模型验证生产 GPIO 初始化时受控电源先保持低电平。
 */
#include "gpio.h"
#include "test_check.h"

#define TEST_POWER_MASK (POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin)
static uint32_t power_low_written;
static unsigned power_configured;
static unsigned exti_disabled;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, uint32_t state)
{
    if (port == GPIOC && (pins & TEST_POWER_MASK) != 0u)
    {
        // 任一路受控电源不能被 GPIO 初始化提前打开。
        TEST_CHECK(state == GPIO_PIN_RESET);
        power_low_written |= pins & TEST_POWER_MASK;
    }
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    if (port == GPIOC && (config->Pin & TEST_POWER_MASK) != 0u)
    {
        // 切成输出前，三路低电平必须已经写入。
        TEST_CHECK(power_low_written == TEST_POWER_MASK);
        TEST_CHECK(config->Mode == GPIO_MODE_OUTPUT_PP);
        ++power_configured;
    }
}

void HAL_NVIC_DisableIRQ(int irq)
{
    TEST_CHECK(irq == EXTI15_10_IRQn);
    ++exti_disabled;
}

int main(void)
{
    MX_GPIO_Init();
    TEST_CHECK(power_low_written == TEST_POWER_MASK);
    TEST_CHECK(power_configured == 1u);
    TEST_CHECK(exti_disabled == 1u);
    return 0;
}
