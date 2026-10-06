#include "lcd_key_input.h"

#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);            \
            return EXIT_FAILURE;                                               \
        }                                                                       \
    } while (0)

int main(void)
{
    LCDKeyInput_t input = {0};
    LCDKeyEvent_t event = {0};

    LCDKeyInputUpdate(&input, 1u, LCD_KEY_MID, 100u, &event);
    CHECK(event.type == LCD_KEY_EVENT_NONE);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_MID, 150u, &event);
    CHECK(event.type == LCD_KEY_EVENT_NONE);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_MID, 200u, &event);
    CHECK(event.type == LCD_KEY_EVENT_DOWN && event.key == LCD_KEY_MID);

    /* A stale/error sample while held cancels the gesture; it must not look like release. */
    LCDKeyInputUpdate(&input, 0u, LCD_KEY_NONE, 1700u, &event);
    CHECK(event.type == LCD_KEY_EVENT_NONE);
    CHECK(input.active == LCD_KEY_NONE && input.candidate == LCD_KEY_NONE);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 1750u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 1800u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 1850u, &event);
    CHECK(event.type == LCD_KEY_EVENT_NONE);

    /* A normal fresh press and release still emits a debounced duration event. */
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_LEFT, 2000u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_LEFT, 2050u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_LEFT, 2100u, &event);
    CHECK(event.type == LCD_KEY_EVENT_DOWN && event.key == LCD_KEY_LEFT);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 3700u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 3750u, &event);
    LCDKeyInputUpdate(&input, 1u, LCD_KEY_NONE, 3800u, &event);
    CHECK(event.type == LCD_KEY_EVENT_RELEASED);
    CHECK(event.key == LCD_KEY_LEFT && event.duration_ms == 1700u);

    puts("PASS: invalid ADC samples cancel LCD key gestures without release actions");
    return EXIT_SUCCESS;
}
