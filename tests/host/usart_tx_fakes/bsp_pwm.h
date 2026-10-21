/** @file bsp_pwm.h
 * @brief 舵机串口发送测试的PWM硬件替身，不执行物理输出。
 */
#ifndef MC02_UART_FAKE_PWM_H
#define MC02_UART_FAKE_PWM_H
#include "tim.h"
typedef struct
{
    float duty;
} PWMInstance;
typedef struct
{
    TIM_HandleTypeDef *htim;
    float period, dutyratio;
} PWM_Init_Config_s;
PWMInstance *PWMRegister(PWM_Init_Config_s *config);
void PWMSetDutyRatio(PWMInstance *instance, float duty);
#endif
