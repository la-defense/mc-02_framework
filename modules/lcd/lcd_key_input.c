/** @file lcd_key_input.c
* @brief 保存按键连续确认次数，并向 LCD 任务报告按下或松开事件。
*/
#include "lcd_key_input.h"

#include <stddef.h>

#define LCD_KEY_DEBOUNCE_COUNT 3u

// 清除等待和按住状态；调用前 input 必须有效。
static void lcd_key_input_reset(LCDKeyInput_t *input)
{
    input->active = LCD_KEY_NONE;
    input->candidate = LCD_KEY_NONE;
    input->press_started_ms = 0u;
    input->candidate_count = 0u;
    input->release_count = 0u;
}

void LCDKeyInputUpdate(LCDKeyInput_t *input, uint8_t sample_valid, LcdKey_e sampled_key,
                       uint32_t now_ms, LCDKeyEvent_t *event)
{
    if (event == NULL)
        return;

    // 先清本次输出，避免没有新事件时再次处理上一次的按键。
    event->type = LCD_KEY_EVENT_NONE;
    event->key = LCD_KEY_NONE;
    event->duration_ms = 0u;
    if (input == NULL)
        return;

    if (!sample_valid || sampled_key < LCD_KEY_NONE || sampled_key > LCD_KEY_RIGHT)
    {
        // 采样中断后不再延续旧按键，下一次有效按下重新计时。
        lcd_key_input_reset(input);
        return;
    }

    if (input->active == LCD_KEY_NONE)
    {
        if (sampled_key == LCD_KEY_NONE)
        {
            input->candidate = LCD_KEY_NONE;
            input->candidate_count = 0u;
            return;
        }

        if (sampled_key != input->candidate)// 键值变化就重新累计，滤掉短暂跳动。
        {
            input->candidate = sampled_key;
            input->candidate_count = 1u;
            return;
        }

        if (++input->candidate_count < LCD_KEY_DEBOUNCE_COUNT)// 第三次同键才报告按下。
            return;

        input->active = sampled_key;
        input->press_started_ms = now_ms;
        input->release_count = 0u;
        event->type = LCD_KEY_EVENT_DOWN;
        event->key = sampled_key;
        return;
    }

    if (sampled_key != LCD_KEY_NONE)
    {
        // 已按住时保持原键；其他方向读数不会直接变成新按键。
        input->release_count = 0u;
        return;
    }

    if (++input->release_count < LCD_KEY_DEBOUNCE_COUNT)// 连续三次无键才确认松开。
        return;

    event->type = LCD_KEY_EVENT_RELEASED;
    event->key = input->active;
    event->duration_ms = now_ms - input->press_started_ms;
    lcd_key_input_reset(input);
}
