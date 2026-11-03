/** @file led.h
 * @brief 告警适配层主机测试用 WS2812 LED 声明。
 */
#ifndef MC02_ROBOT_ALARM_TEST_LED_H
#define MC02_ROBOT_ALARM_TEST_LED_H

#include <stdint.h>
#include "platform_resources.h"

typedef enum
{
    LED_STATUS_OK = 0,
    LED_STATUS_ARGUMENT,
    LED_STATUS_UNINITIALIZED,
    LED_STATUS_BUSY,
    LED_STATUS_TRANSFER,
} LEDStatus_t;

LEDStatus_t LEDInit(PlatformRgbPort_t port);
void LEDProcess(void);
LEDStatus_t LEDSetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif
