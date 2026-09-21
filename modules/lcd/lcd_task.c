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

/* 五向摇杆为电阻分压, 板级例程给出的阈值是 12bit ADC, 本机 ADC 为 16bit, 故 x16:
       mid  : 0    ~ 200   -> 0     ~ 3200
       right: 700  ~ 1000  -> 11200 ~ 16000
       left : 1500 ~ 1800  -> 24000 ~ 28800
       up   : 2200 ~ 2500  -> 35200 ~ 40000
       down : 2800 ~ 3500  -> 44800 ~ 56000

   LCD 模组相对画面逆时针旋转 90 度安装, 模组自身的上下左右都要跟着转 90 度
   才是用户看到的方向, 因此:
       用户视角 "上" = 模组原始 right
       用户视角 "下" = 模组原始 left

   P4 页 KEY 行会实时显示原始 ADC 值, 实机按下后若与上表不符, 直接改下面的
   阈值宏即可(不需要改逻辑)。 */
#define ADC16(adc12) ((uint16_t)((adc12) * 16u))

#define LCD_KEY_MID_LO ADC16(0u)
#define LCD_KEY_MID_HI ADC16(200u)
#define LCD_KEY_RIGHT_LO ADC16(700u)
#define LCD_KEY_RIGHT_HI ADC16(1000u)
#define LCD_KEY_LEFT_LO ADC16(1500u)
#define LCD_KEY_LEFT_HI ADC16(1800u)
#define LCD_KEY_UP_LO ADC16(2200u)
#define LCD_KEY_UP_HI ADC16(2500u)
#define LCD_KEY_DOWN_LO ADC16(2800u)
#define LCD_KEY_DOWN_HI ADC16(3500u)

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;
static LcdKey_e key_active = LCD_KEY_NONE;
static uint32_t key_press_ms = 0;

volatile uint8_t lcd_init_done = 0;
volatile uint32_t lcd_heartbeat = 0;
volatile uint16_t lcd_key_raw = 0;
volatile uint8_t lcd_key_last = 0;

static LcdKey_e lcd_key_read(void)
{
    uint16_t raw = BSP_ADCGetRawKey();
    lcd_key_raw = raw;

    /* 采样无效(悬空/未接模组)保护: 不要把 0 附近的浮空值当成按键 */
    if (raw < 100u)
        return LCD_KEY_NONE;

    if (raw >= LCD_KEY_MID_LO && raw < LCD_KEY_MID_HI)
        return LCD_KEY_MID;

    /* 用户视角的上下 = 模组原始 right/left 触点 */
    if (raw >= LCD_KEY_RIGHT_LO && raw < LCD_KEY_RIGHT_HI)
        return LCD_KEY_UP;
    if (raw >= LCD_KEY_LEFT_LO && raw < LCD_KEY_LEFT_HI)
        return LCD_KEY_DOWN;

    return LCD_KEY_NONE;
}

static void lcd_key_update(void)
{
    LcdKey_e key = lcd_key_read();
    uint32_t now = HAL_GetTick();

    if (key == key_active)
        return;

    if (key != LCD_KEY_NONE)
    {
        key_active = key;
        key_press_ms = now;
        lcd_key_last = (uint8_t)key;
        LOGINFO("[lcd] key %u raw=%u", (unsigned)key, (unsigned)lcd_key_raw);
        return;
    }

    /* 松开: 只认短按, 长按不做动作(避免误触翻页); 单击中键 = 冻结/恢复 */
    LcdKey_e released = key_active;
    uint32_t duration = now - key_press_ms;
    key_active = LCD_KEY_NONE;
    lcd_key_last = 0u;

    if (released == LCD_KEY_NONE || duration < 30u || duration > 1000u)
        return;

    switch (released)
    {
    case LCD_KEY_UP:
        lcd_page = (uint8_t)((lcd_page + LCD_PAGE_COUNT - 1u) % LCD_PAGE_COUNT);
        break;
    case LCD_KEY_DOWN:
        lcd_page = (uint8_t)((lcd_page + 1u) % LCD_PAGE_COUNT);
        break;
    case LCD_KEY_MID:
        lcd_frozen = lcd_frozen ? 0u : 1u;
        LCD_UI_SetFrozen(lcd_frozen);
        LOGINFO("[lcd] freeze %s", lcd_frozen ? "on" : "off");
        return;
    default:
        return;
    }

    LCD_UI_DrawStatic(lcd_page);
    LCD_UI_UpdateValues(lcd_page);
    LOGINFO("[lcd] page %u", (unsigned)(lcd_page + 1u));
}

void StartLCDTASK(void const *argument)
{
    (void)argument;

    LCD_UI_Init();
    lcd_init_done = 1;
    LOGINFO("[lcd] init done, page=%u", (unsigned)(lcd_page + 1u));

    for (;;)
    {
        lcd_heartbeat++;
        /* 50ms 采样按键, 200ms 刷新一次数值(5Hz) */
        for (uint8_t i = 0; i < 4u; ++i)
        {
            lcd_key_update();
            osDelay(50);
        }

        if (!lcd_frozen)
            LCD_UI_UpdateValues(lcd_page);
    }
}
