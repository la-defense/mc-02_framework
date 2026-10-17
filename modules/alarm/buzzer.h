/** @file buzzer.h
 * @brief 共用一个 PWM 蜂鸣器，按报警等级选择第一项正在报警的配置。
 */
#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#include "alarm_def.h"
#define BUZZER_DEVICE_CNT 5

#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988

typedef enum
{
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
}octave_e;

typedef struct
{
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
}Buzzer_config_s;

typedef struct
{
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
}BuzzzerInstance;


/** @brief 注册 TIM12 通道 2 和默认最高级报警，初始无声。
 * @return 1 为成功或已初始化，0 为失败；后续注册失败会清理本次 PWM。
 */
uint8_t BuzzerInit(void);
/** @brief 按等级从高到低查找正在报警的对象，跳过未注册项；无报警时保持占空比为 0。
 * @note 先 BuzzerInit()，再由 Daemon 周期调用；PWM 未初始化时直接返回。
 */
void BuzzerTask();
/** @brief 按报警等级注册一项声音配置。
 * @param[in] config 等级 0～4、音调 OCTAVE_1～8，loudness 为正常的 0～1 占空比。
 * @return 成功返回长期有效对象，参数、同级重复或内存失败返回 NULL；初始 ALARM_OFF。
 * @note OCTAVE_1～7 对应 Do～Si，OCTAVE_8 当前不会重设频率，沿用此前定时器值。
 */
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
/** @brief 取得默认最高级报警对象。
 * @return 初始化成功后的内部对象地址，未初始化时为 NULL；不可释放。
 */
BuzzzerInstance *BuzzerGetInstance(void);
/** @brief 改写报警开关，由下一次 BuzzerTask() 选择执行。
 * @param[in,out] buzzer 有效注册对象，不能为 NULL。
 * @param[in] state ALARM_ON 开启，ALARM_OFF 关闭。
 */
void BuzzerSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);
/** @brief 改写音调选择，不直接操作定时器。
 * @param[in,out] buzzer 有效注册对象，不能为 NULL。
 * @param[in] octave OCTAVE_1～7 可重新设置频率；8 沿用已有频率，不在此处检查范围。
 */
void BuzzerSetOctave(BuzzzerInstance *buzzer, octave_e octave);
#endif // !BUZZER_H
