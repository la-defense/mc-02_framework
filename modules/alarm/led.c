/** @file led.c
* @brief 保存唯一报警灯设置，按任务调用次数变化亮度，并转为 WS2812 RGB 输出。
 */
#include "led.h"
#include "bsp_dwt.h"
#include "ws2812.h"
#include "string.h"

static LEDInstance *led;

/**
 * @brief LED灯初始化,初始化底层 WS2812 (SPI6)
 *
 */
void LEDInit()
{
    WS2812_Init();
}

LEDInstance *LEDRegister(LED_config_s *config)
{
    LEDInstance *led_temp = (LEDInstance *)malloc(sizeof(LEDInstance));
    memset(led_temp, 0, sizeof(LEDInstance));

    led_temp->color = config->color;
    led_temp->flash = config->flash;
    led_temp->flash_count = config->flash_count;
    led_temp->alarm_state = ALARM_OFF;

    led = led_temp;
    return led_temp;
}

void LEDSetStatus(LEDInstance *led, AlarmState_e state)
{
    led->alarm_state = state;
}

void LEDSetFlash(LEDInstance *led, color_e color, flash_e flash)
{
    led->color = color;
    led->flash = flash;
}

void LEDSetFlashTime(LEDInstance *led, uint8_t flash_count)
{
    led->flash_count = flash_count;
}

// n、count 跨调用保存；按模式选择计数阈值，设置颜色时不会自动把这些计数清零。
static void LEDShowFlash(color_e color, flash_e flash)
{
    static uint16_t time_temp;
    static uint16_t n, count;
    static int8_t temp, alpha;
    static uint8_t r, g, b;
    uint8_t on = 0;

    if (flash == FLASH_HIGH)
        time_temp = HightTime;
    else if (flash == FLASH_LOW)
        time_temp = LowTime;

    n++;
    if (n >= time_temp || flash == ALWAYS_ON)
    {
        switch (color)
        {
        case RED:
            r = 255; g = 0;   b = 0;
            break;
        case YELLOW:
            r = 255; g = 255; b = 0;
            break;
        case BLUE:
            r = 0;   g = 0;   b = 255;
            break;
        case GREEN:
            r = 0;   g = 255; b = 0;
            break;
        case PINK:
            temp++;
            if (temp > 1)
            {
                temp = 0;
                alpha += 2;
            }
            // alpha 是 int8_t，用其绝对值作为红、蓝亮度，不能按完整 0～255 的变化理解。
            r = (uint8_t)abs(alpha);
            g = 0;
            b = (uint8_t)abs(alpha);
            break;
        default:
            break;
        }
        on = 1;
    }

    if (on)
    {
        WS2812_SetRGB(r, g, b);
    }
    else if (flash != ALWAYS_ON)
    {
        /* 灭灯阶段 */
        WS2812_SetRGB(0, 0, 0);
    }

    // 快速模式还参考 flash_count，慢速模式按两倍计数阈值复位；均保留 count 到后续调用。
    if ((count >= led->flash_count && flash == FLASH_HIGH) || n >= 2 * time_temp)
    {
        n = 0;
        count++;
    }

    if (count >= 4 * time_temp)
    {
        count = 0;
    }
}

void LEDTask()
{
    // 关闭报警时保留粉色变化，表示任务仍在更新；开启时才采用用户指定颜色。
    if (led->alarm_state == ALARM_OFF)
    {
        LEDShowFlash(PINK, ALWAYS_ON);
    }
    else
    {
        LEDShowFlash(led->color, led->flash);
    }
}
