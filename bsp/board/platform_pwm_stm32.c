/** @file platform_pwm_stm32.c
 * @brief 将逻辑 IMU 加热 PWM 映射到 MC-02 的 TIM3 通道 4。
 */
#include "platform_analog.h"
#include "tim.h"

uint8_t PlatformPwmIsBound(PlatformPwmChannel_t channel)
{
    return (uint8_t)(channel == PLATFORM_PWM_IMU_HEATER);
}

uint8_t PlatformPwmSetCompare(PlatformPwmChannel_t channel, uint32_t compare)
{
    if (!PlatformPwmIsBound(channel)) return 0u; // 未绑定时不写定时器。
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, compare); // 沿用原比较值和时序。
    return 1u;
}

uint8_t PlatformPwmStart(PlatformPwmChannel_t channel)
{
    if (!PlatformPwmIsBound(channel)) return 0u; // 未绑定时不启动输出。
    return (uint8_t)(HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) == HAL_OK);
}
