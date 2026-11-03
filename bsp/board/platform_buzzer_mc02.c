/** @file platform_buzzer_mc02.c
 * @brief 将板载蜂鸣器接口绑定到 MC-02 的 TIM12 通道 2。
 */
#include "platform_buzzer.h"
#include "bsp_pwm.h"
#include <stddef.h>

static PWMInstance *buzzer_pwm;

uint8_t PlatformBuzzerInit(PlatformPwmChannel_t channel, uint16_t initial_frequency_hz)
{
    if (channel != PLATFORM_PWM_BUZZER_1 || initial_frequency_hz == 0u)
        return 0u; // 未绑定的通道不可使用，零频率不能换算周期。
    if (buzzer_pwm != NULL)
        return 1u; // 复用已注册的通道。

    PWM_Init_Config_s config = {
        .htim = &htim12, // 板载蜂鸣器定时器。
        .channel = TIM_CHANNEL_2, // 板载蜂鸣器输出通道。
        .dutyratio = 0.0f, // 先以静音占空比启动。
        .period = 1.0f / (float)initial_frequency_hz, // 将 Hz 换成周期秒数。
    };
    buzzer_pwm = PWMRegister(&config); // 原驱动在注册成功时启动 PWM。
    return buzzer_pwm != NULL ? 1u : 0u;
}

void PlatformBuzzerSetTone(uint16_t frequency_hz, uint16_t duty_permille)
{
    PWMSetPeriod(buzzer_pwm, 1.0f / (float)frequency_hz); // 周期单位为秒。
    PWMSetDutyRatio(buzzer_pwm, (float)duty_permille / 1000.0f); // 千分比换成 0～1。
}

void PlatformBuzzerSilence(void)
{
    PWMSetDutyRatio(buzzer_pwm, 0.0f); // 保持 PWM 已注册并关闭声音。
}
