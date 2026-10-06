#include "lcd_key_input.h"

#include <stddef.h>

#define LCD_KEY_DEBOUNCE_COUNT 3u

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

    event->type = LCD_KEY_EVENT_NONE;
    event->key = LCD_KEY_NONE;
    event->duration_ms = 0u;
    if (input == NULL)
        return;

    if (!sample_valid || sampled_key < LCD_KEY_NONE || sampled_key > LCD_KEY_RIGHT)
    {
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

        if (sampled_key != input->candidate)
        {
            input->candidate = sampled_key;
            input->candidate_count = 1u;
            return;
        }

        if (++input->candidate_count < LCD_KEY_DEBOUNCE_COUNT)
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
        input->release_count = 0u;
        return;
    }

    if (++input->release_count < LCD_KEY_DEBOUNCE_COUNT)
        return;

    event->type = LCD_KEY_EVENT_RELEASED;
    event->key = input->active;
    event->duration_ms = now_ms - input->press_started_ms;
    lcd_key_input_reset(input);
}
