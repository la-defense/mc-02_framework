/** @file buzzer.h
 * @brief 告警适配层主机测试用 PWM 蜂鸣器声明。
 */
#ifndef MC02_ROBOT_ALARM_TEST_BUZZER_H
#define MC02_ROBOT_ALARM_TEST_BUZZER_H

#include "robot_alarm.h"

typedef enum
{
    BUZZER_STATUS_OK = 0,
    BUZZER_STATUS_ARGUMENT,
    BUZZER_STATUS_UNINITIALIZED,
    BUZZER_STATUS_RESOURCE,
} BuzzerStatus_t;

BuzzerStatus_t BuzzerInit(void);
BuzzerStatus_t BuzzerSetTone(BuzzerNote_t note, uint16_t duty_permille);
BuzzerStatus_t BuzzerSilence(void);

#endif
