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

/**
 * @brief          设置LED灯颜色和频率,通过 WS2812 输出 RGB
 * @param[in]      color: 红色/黄色/蓝色/绿色/粉色，[RED/YELLOW/BLUE/GREEN/PINK]
 * @param[in]      flash: 快闪/慢闪/常亮，[FLASH_HIGH/FLASH_LOW/ALWAYS_ON]
 * @note           粉色为呼吸灯变化
 * @retval         none
 */
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
            /* 呼吸灯, alpha 在 0~255 间渐变, 映射到 R/B 亮度 */
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
    if (led->alarm_state == ALARM_OFF)
    {
        LEDShowFlash(PINK, ALWAYS_ON);
    }
    else
    {
        LEDShowFlash(led->color, led->flash);
    }
}
