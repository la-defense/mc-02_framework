/** @file lcd_key_input.h
* @brief 将每次按键采样变成按下、松开事件；连续三次相同结果才确认变化。
*/
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
    LcdKey_e active;           // 已确认按住的键。
    LcdKey_e candidate;        // 正在等待连续三次确认的键。
    uint32_t press_started_ms;// 确认按下时的毫秒时间，不含此前的三次确认等待。
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
    uint32_t duration_ms;      // 仅松开事件填写持续时间，单位为 ms。
} LCDKeyEvent_t;

/**
* @brief 更新一次按键状态；无效采样会取消正在等待或已按住的键，不产生松开事件。
* @param[in,out] input 首次使用前清零，之后每次调用保留；NULL 时仅清空 event。
* @param[in] sample_valid 非 0 表示此次采样可用，为 0 时取消当前状态。
* @param[in] sampled_key 本次键值，超出枚举范围时按无效采样处理。
* @param[in] now_ms 当前毫秒计数，须保持同一时间来源。
* @param[out] event 本次事件；先清空，无事件时 type 为 NONE；NULL 时整个函数不处理。
* @note 连续三次同键确认按下，连续三次 NONE 确认松开；按住期间忽略其他方向。
*/
void LCDKeyInputUpdate(LCDKeyInput_t *input, uint8_t sample_valid, LcdKey_e sampled_key,
                      uint32_t now_ms, LCDKeyEvent_t *event);

#endif
