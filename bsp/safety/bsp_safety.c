#include "bsp_safety.h"

#include "main.h"

#if defined(MC02_SAFETY_TRACE_TEST)
extern void BSP_SafetyTestRecordWrite(const volatile uint32_t *reg, uint32_t value);
#define BSP_SAFETY_REG_WRITE(reg, value)              \
    do                                                \
    {                                                 \
        const uint32_t write_value_ = (value);        \
        BSP_SafetyTestRecordWrite(&(reg), write_value_); \
        (reg) = write_value_;                         \
    } while (0)
#else
#define BSP_SAFETY_REG_WRITE(reg, value) ((reg) = (value))
#endif

/* This is the last-chance shutdown path. Keep it limited to direct register
 * writes: it is called when HAL, the scheduler, or the current stack may be
 * unusable. */
void BSP_SafetyLatchOutputsOff(void)
{
    const uint32_t heater_pin = 1u << 1u; /* PB1 / TIM3_CH4 */
    const uint32_t controlled_power_pins = POWER_24V_2_Pin |
                                           POWER_24V_1_Pin |
                                           POWER_5V_Pin;

    __disable_irq();
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    (void)RCC->APB1LENR;

    /* Force the output data low before switching PB1 away from TIM3. */
    BSP_SAFETY_REG_WRITE(GPIOB->BSRR, heater_pin << 16u);
    GPIOB->OTYPER &= ~heater_pin;
    GPIOB->OSPEEDR &= ~(3u << (1u * 2u));
    GPIOB->PUPDR &= ~(3u << (1u * 2u));
    BSP_SAFETY_REG_WRITE(GPIOB->MODER,
                         (GPIOB->MODER & ~(3u << (1u * 2u))) |
                             (1u << (1u * 2u)));

    BSP_SAFETY_REG_WRITE(GPIOC->BSRR, controlled_power_pins << 16u);
    GPIOC->OTYPER &= ~controlled_power_pins;
    GPIOC->OSPEEDR &= ~((3u << (13u * 2u)) |
                        (3u << (14u * 2u)) |
                        (3u << (15u * 2u)));
    GPIOC->PUPDR &= ~((3u << (13u * 2u)) |
                      (3u << (14u * 2u)) |
                      (3u << (15u * 2u)));
    BSP_SAFETY_REG_WRITE(GPIOC->MODER,
                         (GPIOC->MODER & ~((3u << (13u * 2u)) |
                                          (3u << (14u * 2u)) |
                                          (3u << (15u * 2u)))) |
                             (1u << (13u * 2u)) |
                             (1u << (14u * 2u)) |
                             (1u << (15u * 2u)));

    /* PB1 is now driven low by GPIO; also stop the peripheral source. */
    TIM3->CCR4 = 0u;
    TIM3->CCER &= ~TIM_CCER_CC4E;
    TIM3->CR1 &= ~TIM_CR1_CEN;

    __DSB();
}

void BSP_SafetyStartupLatchOutputsOff(void)
{
    const uint32_t previous_primask = __get_PRIMASK();

    BSP_SafetyLatchOutputsOff();
    __set_PRIMASK(previous_primask);
}
