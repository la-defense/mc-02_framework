/** @file buzzer.c
 * @brief 管理分级报警配置并共用一个 PWM 输出，第一项正在报警的对象优先发声。
 */
#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};
static BuzzzerInstance *buzzer_default;

/**
 * @brief 蜂鸣器初始化
 *
 */
uint8_t BuzzerInit(void)
{
    if (buzzer != NULL && buzzer_default != NULL)
        return 1u;

    PWM_Init_Config_s pwm_config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0.0f,
        .period = 0.001f,
    };
    PWMInstance *pwm = PWMRegister(&pwm_config);
    if (pwm == NULL)
        return 0u;

    Buzzer_config_s default_config = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .octave = OCTAVE_1,
        .loudness = 0.4f,
    };
    BuzzzerInstance *default_instance = BuzzerRegister(&default_config);
    if (default_instance == NULL)
    {
        // 默认报警对象分配失败，撤销本次 PWM 注册，不留下部分初始化的输出。
        (void)PWMUnregister(pwm);
        return 0u;
    }

    buzzer = pwm;
    buzzer_default = default_instance;
    return 1u;
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL || config->alarm_level < 0 ||
        config->alarm_level >= BUZZER_DEVICE_CNT ||
        config->octave < OCTAVE_1 || config->octave > OCTAVE_8 ||
        !isfinite(config->loudness) || config->loudness < 0.0f || config->loudness > 1.0f)
        return NULL;
    if (buzzer_list[config->alarm_level] != NULL)
        return NULL;

    BuzzzerInstance *instance = (BuzzzerInstance *)malloc(sizeof(*instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(*instance));
    instance->alarm_level = config->alarm_level;
    instance->loudness = config->loudness;
    instance->octave = config->octave;
    instance->alarm_state = ALARM_OFF;
    buzzer_list[config->alarm_level] = instance;
    return instance;
}

BuzzzerInstance *BuzzerGetInstance(void)
{
    return buzzer_default;
}

void BuzzerSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void BuzzerSetOctave(BuzzzerInstance *buzzer, octave_e octave)
{
    buzzer->octave = octave;
}

void BuzzerTask()
{
    if (buzzer == NULL)
        return;
    BuzzzerInstance *buzz;
    // 下标越小等级越高，未注册项跳过；处理首个开启项后立即结束本轮选择。
    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz == NULL)
            continue;
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave)
            {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                break;
            default:
                break;
            }
            break;
        }
    }
}
