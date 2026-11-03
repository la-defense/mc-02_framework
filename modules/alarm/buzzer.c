/** @file buzzer.c
 * @brief 通过平台蜂鸣器输出播放告警音符；OCTAVE_8 对应下一组 Do。
 * @note 初始化发生在任务启动前，任务运行时只改周期和比较值。
 */
#include "buzzer.h"
#include "platform_buzzer.h"

static uint8_t buzzer_initialized;
static PlatformPwmChannel_t buzzer_channel;

static const uint16_t buzzer_note_frequency_hz[BUZZER_NOTE_COUNT] = {
    0u, // 静音不使用频率。
    523u, // Do，Hz。
    587u, // Re，Hz。
    659u, // Mi，Hz。
    698u, // Fa，Hz。
    784u, // So，Hz。
    880u, // La，Hz。
    988u, // Si，Hz。
    1046u}; // 高音 Do，Hz。

BuzzerStatus_t BuzzerInit(PlatformPwmChannel_t channel)
{
    if (buzzer_initialized != 0u)
        return channel == buzzer_channel ? BUZZER_STATUS_OK : BUZZER_STATUS_RESOURCE;
    if (PlatformBuzzerInit(channel, buzzer_note_frequency_hz[OCTAVE_1]) == 0u)
        return BUZZER_STATUS_RESOURCE;
    buzzer_channel = channel;
    buzzer_initialized = 1u;
    return BUZZER_STATUS_OK;
}

BuzzerStatus_t BuzzerSetTone(BuzzerNote_t note, uint16_t duty_permille)
{
    if (buzzer_initialized == 0u)
        return BUZZER_STATUS_UNINITIALIZED;
    if (note <= BUZZER_SILENT || note >= BUZZER_NOTE_COUNT ||
        duty_permille == 0u || duty_permille > 1000u)
        return BUZZER_STATUS_ARGUMENT;

    const uint16_t frequency_hz = buzzer_note_frequency_hz[note];
    if (frequency_hz == 0u)
        return BUZZER_STATUS_ARGUMENT;

    PlatformBuzzerSetTone(frequency_hz, duty_permille);
    return BUZZER_STATUS_OK;
}

BuzzerStatus_t BuzzerSilence(void)
{
    if (buzzer_initialized == 0u)
        return BUZZER_STATUS_UNINITIALIZED;
    PlatformBuzzerSilence();
    return BUZZER_STATUS_OK;
}
