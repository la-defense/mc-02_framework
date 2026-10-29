/** @file ws2812.h
 * @brief 为 SPI1／SPI2 主机测试提供不占用句柄的 SPI6 回调替身。
 * @note 实际 SPI6 路由另由生产 WS2812 接口测试验证。
 */
#ifndef MC02_SPI_TEST_WS2812_H
#define MC02_SPI_TEST_WS2812_H
#include "spi.h"
uint8_t WS2812_OnTxComplete(SPI_HandleTypeDef *handle);
uint8_t WS2812_OnError(SPI_HandleTypeDef *handle);
#endif
