#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "bsp_safety.h"

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
typedef enum
{
    TEST_WRITE_HEATER_BSRR,
    TEST_WRITE_HEATER_MODER,
    TEST_WRITE_RAILS_BSRR,
    TEST_WRITE_RAILS_MODER,
} TestWriteEvent_e;

static TestWriteEvent_e write_events[4];
static uint32_t write_event_count;

void BSP_SafetyTestRecordWrite(const volatile uint32_t *reg, uint32_t value)
{
    (void)value;
    if (reg == &mc02_test_gpiob.BSRR)
        write_events[write_event_count++] = TEST_WRITE_HEATER_BSRR;
    else if (reg == &mc02_test_gpiob.MODER)
        write_events[write_event_count++] = TEST_WRITE_HEATER_MODER;
    else if (reg == &mc02_test_gpioc.BSRR)
        write_events[write_event_count++] = TEST_WRITE_RAILS_BSRR;
    else if (reg == &mc02_test_gpioc.MODER)
        write_events[write_event_count++] = TEST_WRITE_RAILS_MODER;
}

static void ResetRegisters(void)
{
    mc02_test_gpiob = (GPIO_TypeDef){0};
    mc02_test_gpioc = (GPIO_TypeDef){0};
    mc02_test_tim3 = (TIM_TypeDef){0};
    mc02_test_rcc = (RCC_TypeDef){0};
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    write_event_count = 0u;
}

static int TestShutdownLatchesLowBeforeDisconnectingPwm(void)
{
    const uint32_t heater_mask = 1u << 1;
    const uint32_t controlled_power_mask = (1u << 13) | (1u << 14) | (1u << 15);
    const uint32_t heater_mode_mask = 3u << (1u * 2u);
    const uint32_t controlled_power_mode_mask = (3u << (13u * 2u)) |
                                                (3u << (14u * 2u)) |
                                                (3u << (15u * 2u));

    ResetRegisters();
    mc02_test_gpiob.ODR = heater_mask;
    mc02_test_gpiob.MODER |= 2u << (1u * 2u); /* PB1 is still on its alternate function. */
    mc02_test_gpioc.ODR = controlled_power_mask;
    mc02_test_gpioc.MODER |= controlled_power_mode_mask;
    mc02_test_gpioc.OTYPER = controlled_power_mask;
    mc02_test_gpioc.PUPDR = controlled_power_mode_mask;
    mc02_test_tim3.CCR4 = 125u;
    mc02_test_tim3.CCER = TIM_CCER_CC4E;
    mc02_test_tim3.CR1 = TIM_CR1_CEN;

    BSP_SafetyLatchOutputsOff();

    CHECK(mc02_test_irq_disabled == 1u);
    CHECK((mc02_test_rcc.AHB4ENR & (RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN)) ==
          (RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN));
    CHECK((mc02_test_rcc.APB1LENR & RCC_APB1LENR_TIM3EN) != 0u);
    CHECK(mc02_test_gpiob.BSRR == (heater_mask << 16u));
    CHECK(mc02_test_gpioc.BSRR == (controlled_power_mask << 16u));
    CHECK(write_event_count == 4u);
    CHECK(write_events[0] == TEST_WRITE_HEATER_BSRR);
    CHECK(write_events[1] == TEST_WRITE_HEATER_MODER);
    CHECK(write_events[2] == TEST_WRITE_RAILS_BSRR);
    CHECK(write_events[3] == TEST_WRITE_RAILS_MODER);
    CHECK((mc02_test_gpiob.MODER & heater_mode_mask) == (1u << (1u * 2u)));
    CHECK((mc02_test_gpioc.MODER & controlled_power_mode_mask) ==
          ((1u << (13u * 2u)) | (1u << (14u * 2u)) | (1u << (15u * 2u))));
    CHECK((mc02_test_gpioc.OTYPER & controlled_power_mask) == 0u);
    CHECK((mc02_test_gpioc.PUPDR & controlled_power_mode_mask) == 0u);
    CHECK(mc02_test_tim3.CCR4 == 0u);
    CHECK((mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u);
    CHECK((mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u);
    return 0;
}

static int TestStartupLatchKeepsInterruptsMasked(void)
{
    ResetRegisters();
    mc02_test_primask = 1u;
    mc02_test_irq_disabled = 1u;
    mc02_test_gpiob.MODER = 2u << (1u * 2u);
    mc02_test_gpioc.MODER = 2u << (13u * 2u);
    mc02_test_tim3.CCR4 = 50u;
    mc02_test_tim3.CCER = TIM_CCER_CC4E;
    mc02_test_tim3.CR1 = TIM_CR1_CEN;

    BSP_SafetyStartupLatchOutputsOff();

    CHECK(mc02_test_primask == 1u);
    CHECK(mc02_test_irq_disabled == 1u);
    CHECK(mc02_test_tim3.CCR4 == 0u);
    CHECK((mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u);
    CHECK((mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u);
    return 0;
}
static int TestStartupLatchRestoresInterruptMask(void)
{
    ResetRegisters();
    mc02_test_gpiob.ODR = 1u << 1u;
    mc02_test_gpiob.MODER = 2u << (1u * 2u);
    mc02_test_gpioc.ODR = (1u << 13u) | (1u << 14u) | (1u << 15u);
    mc02_test_tim3.CCR4 = 250u;
    mc02_test_tim3.CCER = TIM_CCER_CC4E;
    mc02_test_tim3.CR1 = TIM_CR1_CEN;

    BSP_SafetyStartupLatchOutputsOff();

    CHECK(mc02_test_primask == 0u);
    CHECK(mc02_test_irq_disabled == 0u);
    CHECK(write_event_count == 4u);
    CHECK(write_events[0] == TEST_WRITE_HEATER_BSRR);
    CHECK(write_events[1] == TEST_WRITE_HEATER_MODER);
    CHECK(write_events[2] == TEST_WRITE_RAILS_BSRR);
    CHECK(write_events[3] == TEST_WRITE_RAILS_MODER);
    CHECK(mc02_test_tim3.CCR4 == 0u);
    CHECK((mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u);
    CHECK((mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u);
    return 0;
}

int main(void)
{
    if (TestShutdownLatchesLowBeforeDisconnectingPwm() != 0)
        return 1;
    if (TestStartupLatchKeepsInterruptsMasked() != 0)
        return 1;
    if (TestStartupLatchRestoresInterruptMask() != 0)
        return 1;

    puts("PASS: emergency output shutdown disables heater PWM and controlled rails");
    return 0;
}
