#ifndef MC02_HOST_HEATER_FAKE_MAIN_H
#define MC02_HOST_HEATER_FAKE_MAIN_H

#include <stdint.h>
typedef struct { uint32_t compare; } TIM_HandleTypeDef;
uint32_t HAL_GetTick(void);
extern TIM_HandleTypeDef htim3;
void HostSetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
#define __HAL_TIM_SetCompare(timer, channel, value) HostSetCompare((timer), (channel), (value))

#endif
