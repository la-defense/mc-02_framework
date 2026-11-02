/** @file board_spi_stm32.h
 * @brief MC-02 板级 SPI 编号到 STM32 控制器的映射，仅供平台后端使用。
 */
#ifndef MC02_BOARD_SPI_STM32_H
#define MC02_BOARD_SPI_STM32_H
#include "platform_resources.h"
#include "spi.h"

/** @brief 解析本板逻辑 SPI 编号。
 * @param[in] bus 逻辑总线编号。
 * @return 已绑定控制器的长期句柄；未绑定返回 NULL，不表示设备已初始化。
 */
SPI_HandleTypeDef *BoardSpiResolve(PlatformSpiBus_t bus);
#endif
