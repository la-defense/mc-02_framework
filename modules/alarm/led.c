/** @file led.c
 * @brief 将告警策略的颜色请求转交给板载 WS2812 驱动并返回发送结果。
 * @note 本模块不分配内存；真实灯珠波形仍由 MC02-0025 裸板验收。
 */
#include "led.h"
#include "ws2812.h"

static uint8_t led_initialized;

void LEDProcess(void)
{
    if (led_initialized != 0u)
        (void)WS2812_Process();
}

LEDStatus_t LEDInit(void)
{
    const HAL_StatusTypeDef result = WS2812_Init();
    if (result != HAL_OK)
        return result == HAL_BUSY ? LED_STATUS_BUSY : LED_STATUS_TRANSFER;

    led_initialized = 1u;
    return LED_STATUS_OK;
}

LEDStatus_t LEDSetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_initialized == 0u)
        return LED_STATUS_UNINITIALIZED;

    const HAL_StatusTypeDef result = WS2812_SetRGB(r, g, b);
    if (result == HAL_OK)
        return LED_STATUS_OK;
    if (result == HAL_BUSY)
        return LED_STATUS_BUSY;
    return LED_STATUS_TRANSFER;
}
