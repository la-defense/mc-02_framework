/** @file buzzer.h
 * @brief 提供告警策略使用的单路 PWM 蜂鸣器接口。
 * @note RobotAlarmInit() 在任务启动前调用；运行时由 RobotAlarmTask() 串行驱动。
 *       不要从其他调用者并发改写同一 PWM。
 */
#ifndef MC02_ALARM_BUZZER_H
#define MC02_ALARM_BUZZER_H

#include <stdint.h>
#include "robot_alarm.h"
#include "platform_resources.h"

typedef enum
{
    BUZZER_STATUS_OK = 0,
    BUZZER_STATUS_ARGUMENT,
    BUZZER_STATUS_UNINITIALIZED,
    BUZZER_STATUS_RESOURCE,
} BuzzerStatus_t;

/** @brief 初始化板级蜂鸣器输出并以静音状态启动。
 * @param[in] channel 车型声明的逻辑 PWM 通道。
 * @return 成功或已初始化为 BUZZER_STATUS_OK，PWM 注册失败为 RESOURCE。
 * @note 初始化只创建一个 PWM 实例；失败不影响整车 safety 状态。
 */
BuzzerStatus_t BuzzerInit(PlatformPwmChannel_t channel);
/** @brief 设置一个音符和占空比。
 * @param[in] note OCTAVE_1～OCTAVE_8，分别为 Do、Re、Mi、Fa、So、La、Si、高音 Do。
 * @param[in] duty_permille PWM 占空比，范围 1～1000。
 * @return 参数正确且 PWM 已注册为 OK；否则返回参数或初始化错误。
 * @note 仅由 RobotAlarmTask() 所在普通任务串行调用；PWM 接口不返回运行时 HAL 状态。
 */
BuzzerStatus_t BuzzerSetTone(BuzzerNote_t note, uint16_t duty_permille);
/** @brief 关闭蜂鸣器输出。
 * @return PWM 已注册时为 OK，否则为未初始化错误。
 */
BuzzerStatus_t BuzzerSilence(void);

#endif /* MC02_ALARM_BUZZER_H */
