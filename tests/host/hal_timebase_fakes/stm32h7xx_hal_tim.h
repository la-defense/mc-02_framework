#ifndef MC02_HOST_HAL_TIMEBASE_FAKE_TIM_H
#define MC02_HOST_HAL_TIMEBASE_FAKE_TIM_H

#include "stm32h7xx_hal.h"

#define TIM_IT_UPDATE (1u << 0)
#define __HAL_TIM_DISABLE_IT(handle, interrupt) ((handle)->Instance->DIER &= ~(interrupt))
#define __HAL_TIM_ENABLE_IT(handle, interrupt) ((handle)->Instance->DIER |= (interrupt))

HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *handle);

#endif
