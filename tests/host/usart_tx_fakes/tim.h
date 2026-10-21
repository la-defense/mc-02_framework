/** @file tim.h
 * @brief UART发送调用方测试不访问定时器，只保留PWM引用类型。
 */
#ifndef MC02_UART_FAKE_TIM_H
#define MC02_UART_FAKE_TIM_H
typedef struct
{
    int unused;
} TIM_HandleTypeDef;
#endif
