#ifndef LED_H
#define LED_H
#include "stdint.h"
#include "math.h"
#include "alarm_def.h"

// 闪烁时间间隔，Daemon任务频率为100Hz，故单位为10ms
#define  HightTime  20
#define  LowTime  30

typedef enum
{
    FLASH_HIGH = 0,
    FLASH_LOW,
    ALWAYS_ON
}flash_e;

typedef enum
{
    RED = 0,
    YELLOW,
    BLUE,
    GREEN,
    PINK
}color_e;

typedef struct
{
    flash_e flash;
    color_e color;
    uint8_t flash_count;
}LED_config_s;

typedef struct
{
    color_e color;
    flash_e flash;
    uint8_t flash_count;
    AlarmState_e alarm_state;
}LEDInstance;


void LEDInit();
void LEDTask();
LEDInstance *LEDRegister(LED_config_s *config);
void LEDSetStatus(LEDInstance *led, AlarmState_e state);
void LEDSetFlash(LEDInstance *led, color_e color, flash_e flash);
void LEDSetFlashTime(LEDInstance *led, uint8_t flash_count);
#endif // !LED_H
