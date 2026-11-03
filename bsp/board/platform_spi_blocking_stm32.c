/** @file platform_spi_blocking_stm32.c
 * @brief 将框架同步 SPI 传输映射到 MC-02 已注册设备。
 */
#include "../spi/platform_spi_blocking.h"
#include "bsp_spi.h"
#include <stddef.h>

PlatformSpiStatus_t PlatformSpiTransferBlocking(void *device, uint8_t *rx,
                                                uint8_t *tx, uint8_t length)
{
    SPIInstance *spi = device; // 板级适配持有实际 SPI 类型。
    if (spi == NULL || rx == NULL || tx == NULL || length == 0u || spi->spi_work_mode != SPI_BLOCK_MODE)
        return PLATFORM_SPI_ERROR; // 栈上缓冲不能用于异步传输。
    switch (SPITransRecv(spi, rx, tx, length)) // 复用现有总线占用及超时处理。
    {
    case HAL_OK: return PLATFORM_SPI_OK; // 传输完成。
    case HAL_BUSY: return PLATFORM_SPI_BUSY; // 总线或调用上下文暂时不可用。
    case HAL_TIMEOUT: return PLATFORM_SPI_TIMEOUT; // 等待或传输超时。
    default: return PLATFORM_SPI_ERROR; // 其他 HAL 错误统一按传输失败处理。
    }
}
