/** @file board_spi_stm32.c
 * @brief MC-02 的逻辑 SPI 总线绑定。
 */
#include "board_spi_stm32.h"
#include <stddef.h>

SPI_HandleTypeDef *BoardSpiResolve(PlatformSpiBus_t bus)
{
    switch (bus) {
    case PLATFORM_SPI_BUS_1: return &hspi1; // 逻辑总线 1 对应本板 SPI1。
    case PLATFORM_SPI_BUS_2: return &hspi2; // 逻辑总线 2 对应本板 SPI2。
    default: return NULL; // 无效或未绑定编号不能作为设备总线。
    }
}
