/** @file buzzer.c
 * @brief 用一个 TIM12 PWM 实例播放告警音符；OCTAVE_8 对应下一组 Do。
 * @note 初始化发生在任务启动前，任务运行时只改周期和比较值。
 */
#include "buzzer.h"
#include "bsp_pwm.h"

#include <stddef.h>

static PWMInstance *buzzer_pwm;

static const uint16_t buzzer_note_frequency_hz[BUZZER_NOTE_COUNT] = {
    0u, 523u, 587u, 659u, 698u, 784u, 880u, 988u, 1046u};

BuzzerStatus_t BuzzerInit(void)
{
    if (buzzer_pwm != NULL)
        return BUZZER_STATUS_OK;

    PWM_Init_Config_s config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0.0f,
        .period = 1.0f / (float)buzzer_note_frequency_hz[OCTAVE_1],
    };
    buzzer_pwm = PWMRegister(&config);
    return buzzer_pwm != NULL ? BUZZER_STATUS_OK : BUZZER_STATUS_RESOURCE;
}

BuzzerStatus_t BuzzerSetTone(BuzzerNote_t note, uint16_t duty_permille)
{
    if (buzzer_pwm == NULL)
        return BUZZER_STATUS_UNINITIALIZED;
    if (note <= BUZZER_SILENT || note >= BUZZER_NOTE_COUNT ||
        duty_permille == 0u || duty_permille > 1000u)
        return BUZZER_STATUS_ARGUMENT;

    const uint16_t frequency_hz = buzzer_note_frequency_hz[note];
    if (frequency_hz == 0u)
        return BUZZER_STATUS_ARGUMENT;

    PWMSetPeriod(buzzer_pwm, 1.0f / (float)frequency_hz);
    PWMSetDutyRatio(buzzer_pwm, (float)duty_permille / 1000.0f);
    return BUZZER_STATUS_OK;
}

BuzzerStatus_t BuzzerSilence(void)
{
    if (buzzer_pwm == NULL)
        return BUZZER_STATUS_UNINITIALIZED;
    PWMSetDutyRatio(buzzer_pwm, 0.0f);
    return BUZZER_STATUS_OK;
}
