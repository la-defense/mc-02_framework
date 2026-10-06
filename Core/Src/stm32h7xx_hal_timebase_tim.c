/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "bsp_hal_tick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim23;
static volatile uint8_t hal_tick_running;
static volatile uint8_t hal_tick_suspend_pending;
static volatile uint16_t hal_tick_blocking_wait_leases;

static uint32_t HAL_TickEnterCritical(void)
{
  uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();
  return previous_primask;
}

static void HAL_TickExitCritical(uint32_t previous_primask)
{
  __set_PRIMASK(previous_primask);
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM23 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock;

  uint32_t              uwPrescalerValue;
  uint32_t              pFLatency;
/*Configure the TIM23 IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
  HAL_NVIC_SetPriority(TIM23_IRQn, TickPriority ,0U);

  /* Enable the TIM23 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM23_IRQn);
    uwTickPrio = TickPriority;
    }
  else
  {
    return HAL_ERROR;
  }

  /* Enable TIM23 clock */
  __HAL_RCC_TIM23_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM23 clock */
      uwTimclock = 2*HAL_RCC_GetPCLK2Freq();

  /* Compute the prescaler value to have TIM23 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM23 */
  htim23.Instance = TIM23;

  /* Initialize TIMx peripheral as follow:

  + Period = [(TIM23CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim23.Init.Period = (1000000U / 1000U) - 1U;
  htim23.Init.Prescaler = uwPrescalerValue;
  htim23.Init.ClockDivision = 0;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&htim23) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    uint32_t previous_primask = HAL_TickEnterCritical();
    HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&htim23);
    if (status == HAL_OK)
    {
      hal_tick_running = 1u;
      hal_tick_suspend_pending = 0u;
    }
    HAL_TickExitCritical(previous_primask);
    return status;
  }

  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM23 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  uint32_t previous_primask = HAL_TickEnterCritical();
  if (hal_tick_blocking_wait_leases != 0u)
  {
    /* Apply suspension after HAL blocking transfers that rely on this tick. */
    hal_tick_suspend_pending = 1u;
  }
  else
  {
    hal_tick_suspend_pending = 0u;
    hal_tick_running = 0u;
    __HAL_TIM_DISABLE_IT(&htim23, TIM_IT_UPDATE);
  }
  HAL_TickExitCritical(previous_primask);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM23 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  uint32_t previous_primask = HAL_TickEnterCritical();
  hal_tick_suspend_pending = 0u;
  hal_tick_running = 1u;
  __HAL_TIM_ENABLE_IT(&htim23, TIM_IT_UPDATE);
  HAL_TickExitCritical(previous_primask);
}

uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
  uint32_t previous_primask = HAL_TickEnterCritical();
  uint8_t acquired = (uint8_t)(hal_tick_running != 0u &&
                               hal_tick_suspend_pending == 0u &&
                               hal_tick_blocking_wait_leases != UINT16_MAX);
  if (acquired)
    hal_tick_blocking_wait_leases++;
  HAL_TickExitCritical(previous_primask);
  return acquired;
}

void BSP_HALTickReleaseBlockingWait(void)
{
  uint32_t previous_primask = HAL_TickEnterCritical();
  if (hal_tick_blocking_wait_leases != 0u)
  {
    hal_tick_blocking_wait_leases--;
    if (hal_tick_blocking_wait_leases == 0u && hal_tick_suspend_pending != 0u)
    {
      hal_tick_suspend_pending = 0u;
      hal_tick_running = 0u;
      __HAL_TIM_DISABLE_IT(&htim23, TIM_IT_UPDATE);
    }
  }
  HAL_TickExitCritical(previous_primask);
}

