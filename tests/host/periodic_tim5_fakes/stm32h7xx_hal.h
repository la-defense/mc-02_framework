/** @file stm32h7xx_hal.h
 * @brief 模拟 TIM5、RCC 和独立 IRQ；测试不访问任何真实外设。
 */
#ifndef MC02_TEST_PERIODIC_TIM5_HAL_H
#define MC02_TEST_PERIODIC_TIM5_HAL_H
#include <stdint.h>
typedef struct { volatile uint32_t CR1, DIER, SR, EGR, CNT, PSC, ARR, CCR1; } TestTimer_t;
typedef struct { volatile uint32_t CFGR; } TestRcc_t;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
extern TestTimer_t test_timer;
extern TestRcc_t test_rcc;
extern uint8_t test_timer_enabled;
#define TIM5 (&test_timer)
#define RCC (&test_rcc)
#define TIM5_IRQn 50
#define RCC_CFGR_TIMPRE 1u
#define RCC_APB1_DIV1 0u
#define NVIC_PRIORITYGROUP_4 3u
#define TIM_CR1_CEN 1u
#define TIM_EGR_UG 1u
#define TIM_DIER_CC1IE 2u
#define TIM_SR_CC1IF 2u
#define __HAL_RCC_TIM5_IS_CLK_ENABLED() (test_timer_enabled)
#define __HAL_RCC_TIM5_CLK_ENABLE() (test_timer_enabled = 1u)
#define __HAL_RCC_TIM5_CLK_DISABLE() (test_timer_enabled = 0u)
void __HAL_RCC_TIM5_FORCE_RESET(void);
void __HAL_RCC_TIM5_RELEASE_RESET(void);
void __DSB(void);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config, uint32_t *latency);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_SetPriority(int irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
uint32_t NVIC_GetEnableIRQ(int irq);
void NVIC_ClearPendingIRQ(int irq);
#endif
