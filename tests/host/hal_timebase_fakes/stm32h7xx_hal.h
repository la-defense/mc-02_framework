#ifndef MC02_HOST_HAL_TIMEBASE_FAKE_HAL_H
#define MC02_HOST_HAL_TIMEBASE_FAKE_HAL_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct
{
    uint32_t DIER;
} TIM_TypeDef;
typedef struct
{
    TIM_TypeDef *Instance;
    struct
    {
        uint32_t Period;
        uint32_t Prescaler;
        uint32_t ClockDivision;
        uint32_t CounterMode;
    } Init;
} TIM_HandleTypeDef;
typedef struct { uint32_t unused; } RCC_ClkInitTypeDef;

extern TIM_TypeDef mc02_test_tim23;
extern uint32_t uwTickPrio;
extern _Thread_local uint32_t mc02_test_primask;

#define TIM23 (&mc02_test_tim23)
#define TIM23_IRQn 0
#define __NVIC_PRIO_BITS 4u
#define TIM_COUNTERMODE_UP 0u
#define __HAL_RCC_TIM23_CLK_ENABLE() ((void)0)
#define __get_PRIMASK() (mc02_test_primask)
#define __disable_irq() (mc02_test_primask = 1u)
#define __set_PRIMASK(value) (mc02_test_primask = (value))

void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clock, uint32_t *latency);
uint32_t HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t tick_priority);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);

#endif
