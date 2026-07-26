#ifndef WS2812_H
#define WS2812_H

#include "main.h"

/*
 * MC02 板载 WS2812 RGB 彩灯驱动,基于 SPI6 (PA5=SCK, PA7=MOSI).
 * 用 SPI 时序模拟 WS2812 编码: 每个位用 4 个 SPI bit 表示,
 * 0 -> 0xC0 (0b11000000, 高电平时间短), 1 -> 0xF0 (0b11110000, 高电平时间长).
 * 取自达妙 MC02 例程 (资料/例程/CtrBoard-H7_WS2812), 移植到本框架.
 *
 * 注意: SPI6 时钟源为 HSE(24MHz), 分频 4 -> 6MHz, 一个 SPI bit 约 167ns,
 * 4 bit 编码可满足 WS2812 的 T0H/T0L/T1H/T1L 时序要求.
 */

void WS2812_Init(void);
void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif // !WS2812_H
