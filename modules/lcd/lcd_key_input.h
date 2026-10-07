#ifndef LCD_KEY_INPUT_H
#define LCD_KEY_INPUT_H

#include <stdint.h>

typedef enum
{
    LCD_KEY_NONE = 0,
    LCD_KEY_MID,
    LCD_KEY_UP,
    LCD_KEY_DOWN,
    LCD_KEY_LEFT,
    LCD_KEY_RIGHT,
} LcdKey_e;

typedef struct
{
    LcdKey_e active;
    LcdKey_e candidate;
    uint32_t press_started_ms;
    uint8_t candidate_count;
    uint8_t release_count;
} LCDKeyInput_t;

typedef enum
{
    LCD_KEY_EVENT_NONE = 0,
    LCD_KEY_EVENT_DOWN,
    LCD_KEY_EVENT_RELEASED,
} LCDKeyEventType_e;

typedef struct
{
    LCDKeyEventType_e type;
    LcdKey_e key;
    uint32_t duration_ms;
} LCDKeyEvent_t;

void LCDKeyInputUpdate(LCDKeyInput_t *input, uint8_t sample_valid, LcdKey_e sampled_key,
                      uint32_t now_ms, LCDKeyEvent_t *event);

#endif
