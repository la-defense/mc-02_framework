#include "bsp_hal_tick.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"

#include <stdio.h>
#include <stdlib.h>

TIM_TypeDef mc02_test_tim23;
uint32_t uwTickPrio;
_Thread_local uint32_t mc02_test_primask;

void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{ (void)irq; (void)priority; (void)subpriority; }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clock, uint32_t *latency)
{ (void)clock; *latency = 0u; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 1000000u; }
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *handle)
{ (void)handle; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *handle)
{ handle->Instance->DIER |= TIM_IT_UPDATE; return HAL_OK; }

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

int main(void)
{
    if (HAL_InitTick(0u) != HAL_OK)
        fail("HAL tick test setup must initialize the production timebase");
    if ((mc02_test_tim23.DIER & TIM_IT_UPDATE) == 0u)
        fail("initialized HAL tick must enable the TIM23 update interrupt");

    if (!BSP_HALTickTryAcquireBlockingWait())
        fail("active HAL tick must allow a blocking-wait lease");
    HAL_SuspendTick();
    if ((mc02_test_tim23.DIER & TIM_IT_UPDATE) == 0u)
        fail("HAL_SuspendTick must defer interrupt disable while a blocking wait owns a lease");
    if (BSP_HALTickTryAcquireBlockingWait())
        fail("a pending tick suspension must reject new blocking-wait leases");
    BSP_HALTickReleaseBlockingWait();
    if ((mc02_test_tim23.DIER & TIM_IT_UPDATE) != 0u)
        fail("releasing the final lease must apply a deferred tick suspension");
    if (BSP_HALTickTryAcquireBlockingWait())
        fail("a suspended HAL tick must not grant a blocking-wait lease");

    HAL_ResumeTick();
    if (!BSP_HALTickTryAcquireBlockingWait())
        fail("HAL_ResumeTick must make blocking-wait leases available again");
    HAL_SuspendTick();
    HAL_ResumeTick();
    BSP_HALTickReleaseBlockingWait();
    if ((mc02_test_tim23.DIER & TIM_IT_UPDATE) == 0u)
        fail("HAL_ResumeTick must cancel a deferred suspension before lease release");

    HAL_SuspendTick();
    if ((mc02_test_tim23.DIER & TIM_IT_UPDATE) != 0u)
        fail("HAL_SuspendTick must disable the tick when no blocking wait is active");
    HAL_ResumeTick();
    return EXIT_SUCCESS;
}
