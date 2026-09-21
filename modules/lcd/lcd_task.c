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

/* 模组在位检测(热插拔恢复):
   模组拔掉后 PA5(按键分压输入)没有驱动, 重新插上时模组已经掉电复位,
   必须重发初始化序列才可能重新显示。
   实测(2026-09-21, 拔掉模组): PA5 稳定在 ~5900(0.29V), 既不越界也不跳动,
   所以"越界/极差"判据抓不到, 必须按"极性档位带"判断:
     合法档位 = mid(0~3200) / right(11200~16000) / left(24000~28800)
                / up(35200~40000) / down(44800~56000) / 空闲(>=58000)
   落在档位之间的缝隙(如拔掉时的 5900)即视为不在位。 */
#define LCD_PRESENCE_WINDOW 10u
#define LCD_PRESENCE_SPREAD_MAX 1500u
#define LCD_IDLE_RAW_MIN 58000u
#define LCD_ABSENT_RAW_LO 100u
#define LCD_ABSENT_RAW_HI 65200u
#define LCD_PRESENT_CONFIRM 8u  /* 连续 400ms 正常 -> 判定插回 */

/* 短按: 翻页/冻结; 长按中键: 手动重初始化面板(拔插后没自动恢复时的兜底) */
#define LCD_KEY_SHORT_MAX_MS 1000u
#define LCD_KEY_LONG_MIN_MS 1500u

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;
static LcdKey_e key_active = LCD_KEY_NONE;
static uint32_t key_press_ms = 0;

volatile uint8_t lcd_init_done = 0;
volatile uint32_t lcd_heartbeat = 0;
volatile uint16_t lcd_key_raw = 0;
volatile uint8_t lcd_key_last = 0;
volatile uint8_t lcd_module_absent = 0;
volatile uint16_t lcd_key_spread = 0;
volatile uint32_t lcd_recover_count = 0;

static LcdKey_e lcd_key_read(void)
{
    uint16_t raw = BSP_ADCGetRawKey();
    lcd_key_raw = raw;

    /* 采样无效(悬空/未接模组)保护: 不要把 0 附近的浮空值当成按键 */
    if (raw < LCD_ABSENT_RAW_LO)
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

/* 采样是否落在某个合法档位带内(含空闲带)。落在档位之间的缝隙里 = 模组不在位。 */
static uint8_t lcd_raw_valid_level(uint16_t raw)
{
    if (raw > LCD_ABSENT_RAW_HI)
        return 0u;
    if (raw >= LCD_IDLE_RAW_MIN)
        return 1u;
    if (raw < LCD_KEY_MID_HI)
        return 1u;
    if (raw >= LCD_KEY_RIGHT_LO && raw < LCD_KEY_RIGHT_HI)
        return 1u;
    if (raw >= LCD_KEY_LEFT_LO && raw < LCD_KEY_LEFT_HI)
        return 1u;
    if (raw >= LCD_KEY_UP_LO && raw < LCD_KEY_UP_HI)
        return 1u;
    if (raw >= LCD_KEY_DOWN_LO && raw < LCD_KEY_DOWN_HI)
        return 1u;
    return 0u;
}

static void lcd_module_presence_check(void)
{
    static uint16_t hist[LCD_PRESENCE_WINDOW];
    static uint8_t hist_idx = 0;
    static uint8_t hist_filled = 0;
    static uint8_t present_streak = 0;
    static uint8_t invalid_seen = 0;

    uint16_t raw = lcd_key_raw;

    hist[hist_idx] = raw;
    hist_idx = (uint8_t)((hist_idx + 1u) % LCD_PRESENCE_WINDOW);
    if (hist_idx == 0u)
        hist_filled = 1u;

    uint8_t invalid = 0;
    if (!lcd_raw_valid_level(raw))
    {
        invalid = 1u;
    }
    else if (hist_filled)
    {
        uint8_t idle_cnt = 0;
        uint16_t idle_min = 0xFFFFu;
        uint16_t idle_max = 0u;
        for (uint8_t i = 0; i < LCD_PRESENCE_WINDOW; ++i)
        {
            uint16_t s = hist[i];
            if (s >= LCD_IDLE_RAW_MIN)
            {
                idle_cnt++;
                if (s < idle_min)
                    idle_min = s;
                if (s > idle_max)
                    idle_max = s;
            }
        }
        lcd_key_spread = (idle_cnt > 0u) ? (uint16_t)(idle_max - idle_min) : 0u;

        /* 空闲电平自己抖得厉害也说明模组不在位。
           只看空闲采样之间的极差, 否则长按按键时电平本来就是变化的 */
        if (idle_cnt >= 5u && lcd_key_spread > LCD_PRESENCE_SPREAD_MAX)
            invalid = 1u;
    }

    if (invalid)
    {
        /* 只要出现过一次"缝隙档位/剧烈跳动", 就记下这次拔插事件,
           之后电平回到合法档位并稳定 400ms 就重初始化面板。
           这样即使拔插很快(只抓到一两帧), 也能恢复。 */
        invalid_seen = 1u;
        present_streak = 0u;
        if (!lcd_module_absent)
        {
            lcd_module_absent = 1u;
            LOGINFO("[lcd] module absent? raw=%u spread=%u", (unsigned)raw, (unsigned)lcd_key_spread);
        }
    }
    else
    {
        if (present_streak < 0xFFu)
            present_streak++;

        if (invalid_seen && present_streak >= LCD_PRESENT_CONFIRM)
        {
            invalid_seen = 0u;
            present_streak = 0u;
            lcd_module_absent = 0u;
            lcd_recover_count++;
            LOGINFO("[lcd] module back, re-init panel (recover=%lu)", (unsigned long)lcd_recover_count);
            LCD_UI_Recover();
        }
    }
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

    /* 松开: 短按 = 翻页/冻结, 长按中键 = 手动重初始化面板 */
    LcdKey_e released = key_active;
    uint32_t duration = now - key_press_ms;
    key_active = LCD_KEY_NONE;
    lcd_key_last = 0u;

    if (released == LCD_KEY_NONE || duration < 30u)
        return;

    if (duration >= LCD_KEY_LONG_MIN_MS)
    {
        if (released == LCD_KEY_MID)
        {
            lcd_recover_count++;
            LOGINFO("[lcd] manual re-init (recover=%lu)", (unsigned long)lcd_recover_count);
            LCD_UI_Recover();
        }
        return;
    }

    if (duration > LCD_KEY_SHORT_MAX_MS)
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
        /* 50ms 采样按键与模组在位状态, 200ms 刷新一次数值(5Hz) */
        for (uint8_t i = 0; i < 4u; ++i)
        {
            lcd_key_update();
            lcd_module_presence_check();
            osDelay(50);
        }

        if (!lcd_frozen && !lcd_module_absent)
            LCD_UI_UpdateValues(lcd_page);
    }
}
