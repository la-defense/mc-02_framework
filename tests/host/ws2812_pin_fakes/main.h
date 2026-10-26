/**
 * @file main.h
 * @brief 为 WS2812 引脚归属主机测试替代板级 HAL 入口。
 * @note 仅覆盖本测试编译所需的 HAL 声明，不模拟 STM32 引脚电平。
 */
#ifndef MC02_WS2812_TEST_MAIN_H
#define MC02_WS2812_TEST_MAIN_H

#include "stm32h7xx_hal.h"

void Error_Handler(void);

#endif
