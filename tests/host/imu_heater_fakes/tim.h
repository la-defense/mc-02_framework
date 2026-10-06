#ifndef MC02_HOST_HEATER_FAKE_TIM_H
#define MC02_HOST_HEATER_FAKE_TIM_H
#include "main.h"
#define TIM_CHANNEL_4 4u
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
#endif
