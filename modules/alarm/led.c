/** @file led.c
 * @brief 将告警策略的颜色请求交给平台 RGB 接口并返回发送结果。
 * @note 本模块不分配内存；真实灯珠波形仍由 MC02-0025 裸板验收。
 */
#include "led.h"
#include "platform_rgb.h"

static uint8_t led_initialized;

void LEDProcess(void)
{
    if (led_initialized != 0u)
        PlatformRgbProcess();
}

LEDStatus_t LEDInit(void)
{
    const PlatformRgbStatus_t result = PlatformRgbInit();
    if (result != PLATFORM_RGB_OK)
        return result == PLATFORM_RGB_BUSY ? LED_STATUS_BUSY : LED_STATUS_TRANSFER;

    led_initialized = 1u;
    return LED_STATUS_OK;
}

LEDStatus_t LEDSetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_initialized == 0u)
        return LED_STATUS_UNINITIALIZED;

    const PlatformRgbStatus_t result = PlatformRgbSet(r, g, b);
    if (result == PLATFORM_RGB_OK)
        return LED_STATUS_OK;
    if (result == PLATFORM_RGB_BUSY)
        return LED_STATUS_BUSY;
    return LED_STATUS_TRANSFER;
}
