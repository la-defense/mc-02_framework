#include <stdint.h>
#include <stdio.h>

#include "main.h"

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return 1;                                                            \
        }                                                                        \
    } while (0)

GPIO_TypeDef mc02_test_gpiob;
GPIO_TypeDef mc02_test_gpioc;
TIM_TypeDef mc02_test_tim3;
RCC_TypeDef mc02_test_rcc;
uint32_t mc02_test_irq_disabled;
uint32_t mc02_test_primask;

int main(void)
{
    const uint32_t heater_pin = 1u << 1u;
    const uint32_t rail_pins = (1u << 13u) | (1u << 14u) | (1u << 15u);
    const uint32_t heater_mode = 3u << 2u;
    const uint32_t rail_modes = (3u << 26u) | (3u << 28u) | (3u << 30u);

    mc02_test_gpiob.MODER = 2u << 2u;
    mc02_test_gpiob.ODR = heater_pin;
    mc02_test_gpioc.MODER = 2u << 26u | 2u << 28u | 2u << 30u;
    mc02_test_gpioc.ODR = rail_pins;
    mc02_test_tim3.CCR4 = 100u;
    mc02_test_tim3.CCER = TIM_CCER_CC4E;
    mc02_test_tim3.CR1 = TIM_CR1_CEN;

    Error_Handler();

    CHECK(mc02_test_irq_disabled == 1u);
    CHECK(mc02_test_gpiob.BSRR == (heater_pin << 16u));
    CHECK(mc02_test_gpioc.BSRR == (rail_pins << 16u));
    CHECK((mc02_test_gpiob.MODER & heater_mode) == (1u << 2u));
    CHECK((mc02_test_gpioc.MODER & rail_modes) ==
          ((1u << 26u) | (1u << 28u) | (1u << 30u)));
    CHECK(mc02_test_tim3.CCR4 == 0u);
    CHECK((mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u);
    CHECK((mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u);
    puts("PASS: fatal error entry latches production outputs off before test return");
    return 0;
}
