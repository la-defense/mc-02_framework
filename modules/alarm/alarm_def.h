#ifndef ALARM_DEF_H
#define ALARM_DEF_H

// 报警器的一些通用定义

/* 报警等级 */
typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
}AlarmLevel_e;

/* 报警是否开启 */
typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
}AlarmState_e;

#endif // !ALARM_DEF_H
