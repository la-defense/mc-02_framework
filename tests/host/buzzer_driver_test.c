/** @file buzzer_driver_test.c
 * @brief 使用生产蜂鸣器适配层核对 PWM 音高、占空比和初始化失败处理。
 * @note PWM 替身只记录寄存器请求；不测量定时器引脚或实际声压。
 */
#include "test_check.h"
#include "buzzer.h"
#include "bsp_pwm.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

TIM_HandleTypeDef htim12;
static PWMInstance fake_pwm;
static PWM_Init_Config_s registered_config;
static uint32_t register_calls;
static uint32_t period_calls;
static uint32_t duty_calls;
static uint8_t registration_fails;

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    TEST_CHECK(config != NULL);
    ++register_calls;
    registered_config = *config;
    return registration_fails != 0u ? NULL : &fake_pwm;
}

void PWMSetPeriod(PWMInstance *pwm, float period)
{
    TEST_CHECK(pwm == &fake_pwm);
    ++period_calls;
    fake_pwm.period = period;
}

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    TEST_CHECK(pwm == &fake_pwm);
    ++duty_calls;
    fake_pwm.dutyratio = dutyratio;
}

static void TestNoteMapping(void)
{
    static const float expected_hz[BUZZER_NOTE_COUNT] = {
        0.0f, 523.0f, 587.0f, 659.0f, 698.0f, 784.0f, 880.0f, 988.0f, 1046.0f};
    TEST_CHECK(BuzzerInit() == BUZZER_STATUS_OK);
    TEST_CHECK(BuzzerInit() == BUZZER_STATUS_OK);
    TEST_CHECK(register_calls == 1u);
    TEST_CHECK(registered_config.htim == &htim12);
    TEST_CHECK(registered_config.channel == TIM_CHANNEL_2);
    TEST_CHECK(registered_config.dutyratio == 0.0f);

    for (BuzzerNote_t note = OCTAVE_1; note < BUZZER_NOTE_COUNT; ++note)
    {
        TEST_CHECK(BuzzerSetTone(note, 250u) == BUZZER_STATUS_OK);
        const float expected_period = 1.0f / expected_hz[note];
        TEST_CHECK(fabsf(fake_pwm.period - expected_period) < 1.0e-7f);
        TEST_CHECK(fabsf(fake_pwm.dutyratio - 0.25f) < 1.0e-7f);
    }
    TEST_CHECK(BuzzerSilence() == BUZZER_STATUS_OK);
    TEST_CHECK(fake_pwm.dutyratio == 0.0f);
    TEST_CHECK(BuzzerSetTone(BUZZER_SILENT, 100u) == BUZZER_STATUS_ARGUMENT);
    TEST_CHECK(BuzzerSetTone(OCTAVE_1, 0u) == BUZZER_STATUS_ARGUMENT);
    TEST_CHECK(BuzzerSetTone(OCTAVE_1, 1001u) == BUZZER_STATUS_ARGUMENT);
    TEST_CHECK(BuzzerSetTone((BuzzerNote_t)-1, 100u) == BUZZER_STATUS_ARGUMENT);
    TEST_CHECK(period_calls == BUZZER_NOTE_COUNT - 1u);
    TEST_CHECK(duty_calls == BUZZER_NOTE_COUNT);
}

static void TestRegistrationFailure(void)
{
    registration_fails = 1u;
    TEST_CHECK(BuzzerInit() == BUZZER_STATUS_RESOURCE);
    TEST_CHECK(BuzzerSetTone(OCTAVE_1, 100u) == BUZZER_STATUS_UNINITIALIZED);
    registration_fails = 0u;
    TEST_CHECK(BuzzerInit() == BUZZER_STATUS_OK);
    TEST_CHECK(register_calls == 2u);
    TEST_CHECK(BuzzerSetTone(OCTAVE_8, 500u) == BUZZER_STATUS_OK);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "notes") == 0)
        TestNoteMapping();
    else if (strcmp(argv[1], "failure") == 0)
        TestRegistrationFailure();
    else
        TEST_CHECK(0);
    return 0;
}
