#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "string.h"
#include "stdlib.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};
static BuzzzerInstance *buzzer_default;

/**
 * @brief 蜂鸣器初始化
 *
 */
void BuzzerInit()
{
    PWM_Init_Config_s buzzer_config = {
        .htim = &htim12,
        .channel = TIM_CHANNEL_2,
        .dutyratio = 0,
        .period = 0.001,
    };
    buzzer = PWMRegister(&buzzer_config);

    /* 注册一个默认蜂鸣器实例，避免 BuzzerTask 遍历空指针，并供上层报警逻辑调用/测试 */
    Buzzer_config_s default_config = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .octave = OCTAVE_1,
        .loudness = 0.4f,
    };
    buzzer_default = BuzzerRegister(&default_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL)
        return NULL;
    if (config->alarm_level >= BUZZER_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    if (buzzer_temp == NULL)
        return NULL;
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
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
