#ifndef MC02_TIM_PROFILE_FAKE_H
#define MC02_TIM_PROFILE_FAKE_H
#include <stdint.h>
typedef struct { uint32_t ODR, mode, alternate; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioe;
#define GPIOA (&fake_gpioa)
#define GPIOB (&fake_gpiob)
#define GPIOE (&fake_gpioe)
#define TIM1 ((void *)1)
#define TIM2 ((void *)2)
#define TIM3 ((void *)3)
#define TIM12 ((void *)12)
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter, AutoReloadPreload; } TIM_Base_InitTypeDef;
typedef struct { void *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterOutputTrigger2, MasterSlaveMode; } TIM_MasterConfigTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState; } TIM_OC_InitTypeDef;
typedef struct { uint32_t OffStateRunMode, OffStateIDLEMode, LockLevel, DeadTime, BreakState, BreakPolarity, BreakFilter, Break2State, Break2Polarity, Break2Filter, AutomaticOutput; } TIM_BreakDeadTimeConfigTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_AF_PP 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_AF1_TIM1 1u
#define GPIO_AF1_TIM2 1u
#define GPIO_AF2_TIM3 2u
#define GPIO_AF2_TIM12 2u
#define GPIO_PIN_0 (1u << 0u)
#define GPIO_PIN_1 (1u << 1u)
#define GPIO_PIN_2 (1u << 2u)
#define GPIO_PIN_9 (1u << 9u)
#define GPIO_PIN_13 (1u << 13u)
#define GPIO_PIN_15 (1u << 15u)
#define TIM_COUNTERMODE_UP 0u
#define TIM_CLOCKDIVISION_DIV1 1u
#define TIM_AUTORELOAD_PRELOAD_DISABLE 2u
#define TIM_AUTORELOAD_PRELOAD_ENABLE 3u
#define TIM_TRGO_RESET 4u
#define TIM_TRGO2_RESET 5u
#define TIM_MASTERSLAVEMODE_DISABLE 6u
#define TIM_OCMODE_PWM1 7u
#define TIM_OCPOLARITY_HIGH 8u
#define TIM_OCNPOLARITY_HIGH 9u
#define TIM_OCFAST_DISABLE 10u
#define TIM_OCIDLESTATE_RESET 11u
#define TIM_OCNIDLESTATE_RESET 12u
#define TIM_OSSR_DISABLE 13u
#define TIM_OSSI_DISABLE 14u
#define TIM_LOCKLEVEL_OFF 15u
#define TIM_BREAK_DISABLE 16u
#define TIM_BREAKPOLARITY_HIGH 17u
#define TIM_BREAK2_DISABLE 18u
#define TIM_BREAK2POLARITY_HIGH 19u
#define TIM_AUTOMATICOUTPUT_DISABLE 20u
#define TIM_CHANNEL_1 21u
#define TIM_CHANNEL_2 22u
#define TIM_CHANNEL_3 23u
#define TIM_CHANNEL_4 24u
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_DISABLE() ((void)0)
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM2_CLK_DISABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM3_CLK_DISABLE() ((void)0)
#define __HAL_RCC_TIM12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM12_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_DISABLE() ((void)0)
int HAL_TIM_PWM_Init(TIM_HandleTypeDef *);
int HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *, TIM_MasterConfigTypeDef *);
int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *, TIM_OC_InitTypeDef *, uint32_t);
int HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *, TIM_BreakDeadTimeConfigTypeDef *);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, uint32_t);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *);
void Error_Handler(void);
#endif
