/** @file bsp_pwm.h
 * @brief 蜂鸣器主机测试用 PWM 寄存器替身声明。
 * @note 只由独立测试目标使用，不进入固件构建。
 */
#ifndef MC02_ALARM_TEST_BSP_PWM_H
#define MC02_ALARM_TEST_BSP_PWM_H

#include <stdint.h>

typedef struct { uint32_t marker; } TIM_HandleTypeDef;
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float period;
    float dutyratio;
} PWM_Init_Config_s;
typedef struct
{
    float period;
    float dutyratio;
} PWMInstance;

extern TIM_HandleTypeDef htim12;
PWMInstance *PWMRegister(PWM_Init_Config_s *config);
void PWMSetPeriod(PWMInstance *pwm, float period);
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);

#define TIM_CHANNEL_2 2u

#endif
