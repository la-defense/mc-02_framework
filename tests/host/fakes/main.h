#ifndef MC02_HOST_FAKE_MAIN_H
#define MC02_HOST_FAKE_MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;

typedef struct
{
    volatile uint32_t AHB4ENR;
    volatile uint32_t APB1LENR;
} RCC_TypeDef;

extern GPIO_TypeDef mc02_test_gpiob;
extern GPIO_TypeDef mc02_test_gpioc;
extern TIM_TypeDef mc02_test_tim3;
extern RCC_TypeDef mc02_test_rcc;
extern uint32_t mc02_test_irq_disabled;
extern uint32_t mc02_test_primask;
extern uint32_t mc02_test_tick;
void NVIC_SystemReset(void);
void Error_Handler(void);
void BSP_SafetyTestRecordWrite(const volatile uint32_t *reg, uint32_t value);

#define GPIOB (&mc02_test_gpiob)
#define GPIOC (&mc02_test_gpioc)
#define TIM3 (&mc02_test_tim3)
#define RCC (&mc02_test_rcc)

#define RCC_AHB4ENR_GPIOBEN (1u << 1)
#define RCC_AHB4ENR_GPIOCEN (1u << 2)
#define RCC_APB1LENR_TIM3EN (1u << 1)
#define TIM_CCER_CC4E (1u << 12)
#define TIM_CR1_CEN (1u << 0)
#define POWER_24V_2_Pin (1u << 13)
#define POWER_24V_1_Pin (1u << 14)
#define POWER_5V_Pin (1u << 15)

#define __DSB() ((void)0)
#define __disable_irq() (mc02_test_irq_disabled = 1u, mc02_test_primask = 1u)
#define __get_PRIMASK() (mc02_test_primask)
#define __set_PRIMASK(value) (mc02_test_primask = (value), mc02_test_irq_disabled = ((value) & 1u))
#define HAL_GetTick() (mc02_test_tick)

#endif
