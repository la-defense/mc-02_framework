#include "lcd_task.h"
#include "lcd_ui.h"
#include "lcd_buf.h"
#include "bsp_adc.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "main.h"

typedef enum
{
    LCD_KEY_NONE = 0,
    LCD_KEY_MID,
    LCD_KEY_UP,
    LCD_KEY_DOWN,
} LcdKey_e;

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;
static LcdKey_e key_active = LCD_KEY_NONE;
static uint32_t key_press_ms = 0;
static uint8_t key_long_fired = 0;
volatile uint8_t lcd_init_done = 0;
volatile uint32_t lcd_heartbeat = 0;

static LcdKey_e lcd_key_read(void)
{
    uint16_t raw = BSP_ADCGetRawKey();

    /* Startup/invalid sample protection: do not treat a floating 0 as a key. */
    if (raw < 100u)
        return LCD_KEY_NONE;

    /* Example thresholds are for 12-bit ADC; current ADC is 16-bit, scale x16. */
    if (raw < 3200u)
        return LCD_KEY_MID;
    if (raw >= 35200u && raw <= 40000u)
        return LCD_KEY_UP;
    if (raw >= 44800u && raw <= 56000u)
        return LCD_KEY_DOWN;

    return LCD_KEY_NONE;
}

static void lcd_key_update(void)
{
    LcdKey_e key = lcd_key_read();
    uint32_t now = HAL_GetTick();

    if (key != LCD_KEY_NONE && key_active == LCD_KEY_NONE)
    {
        key_active = key;
        key_press_ms = now;
        key_long_fired = 0;
        return;
    }

    if (key != LCD_KEY_NONE && key_active != LCD_KEY_NONE)
    {
        if (!key_long_fired && (uint32_t)(now - key_press_ms) >= 1600u)
        {
            lcd_frozen = lcd_frozen ? 0u : 1u;
            key_long_fired = 1u;
            LCD_UI_SetFrozen(lcd_frozen);
        }
        return;
    }

    if (key == LCD_KEY_NONE && key_active != LCD_KEY_NONE)
    {
        uint32_t duration = now - key_press_ms;
        if (!key_long_fired && duration < 800u)
        {
            if (key_active == LCD_KEY_UP)
                lcd_page = (uint8_t)((lcd_page + LCD_PAGE_COUNT - 1u) % LCD_PAGE_COUNT);
            else
                lcd_page = (uint8_t)((lcd_page + 1u) % LCD_PAGE_COUNT);

            LCD_UI_DrawStatic(lcd_page);
            LCD_UI_UpdateValues(lcd_page);
        }
        key_active = LCD_KEY_NONE;
        key_press_ms = 0;
        key_long_fired = 0;
    }
}

void StartLCDTASK(void const *argument)
{
    (void)argument;

    LCD_UI_Init();
    lcd_init_done = 1;
    LOGINFO("[lcd] init done, page=%u", (unsigned)lcd_page);

    for (;;)
    {
        lcd_heartbeat++;
        /* 50ms key sampling, 200ms value refresh (5Hz). */
        for (uint8_t i = 0; i < 4u; ++i)
        {
            lcd_key_update();
            osDelay(50);
        }

        if (!lcd_frozen)
            LCD_UI_UpdateValues(lcd_page);
    }
}
