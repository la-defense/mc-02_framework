#include <stdio.h>
#include "tim.h"
GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioe;
static unsigned writes, inits, unsafe_init;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, uint32_t state)
{
    if (state == GPIO_PIN_RESET) port->ODR &= ~pins;
    else port->ODR |= pins;
    ++writes;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{
    if (port == GPIOB && config->Pin == GPIO_PIN_1 &&
        config->Mode == GPIO_MODE_OUTPUT_PP && (port->ODR & GPIO_PIN_1))
        ++unsafe_init;
    port->mode = config->Mode;
    port->alternate = config->Alternate;
    ++inits;
}
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)
int main(void)
{
    TIM_HandleTypeDef timer = {.Instance = TIM3};
    for (unsigned repeat = 0; repeat < 2; ++repeat)
    {
        fake_gpiob.ODR = GPIO_PIN_1; /* Also exercise reinitialization from a high latch. */
        writes = inits = unsafe_init = 0;
        HAL_TIM_MspPostInit(&timer);
#if MC02_HEATER_ENABLED
        CHECK(fake_gpiob.mode == GPIO_MODE_AF_PP);
        CHECK(fake_gpiob.alternate == GPIO_AF2_TIM3);
        CHECK(writes == 0);
#else
        CHECK(fake_gpiob.mode == GPIO_MODE_OUTPUT_PP);
        CHECK((fake_gpiob.ODR & GPIO_PIN_1) == 0);
        CHECK(writes == 1);
        CHECK(unsafe_init == 0);
#endif
        CHECK(inits == 1);
    }
    timer.Instance = TIM12;
    HAL_TIM_MspPostInit(&timer);
    CHECK(fake_gpiob.mode == GPIO_MODE_AF_PP);
    CHECK(fake_gpiob.alternate == GPIO_AF2_TIM12);
    puts("PASS: production TIM post-init respects heater permission and repeated initialization");
    return 0;
}
