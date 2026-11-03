/** @file platform_spi_register_stm32.c
 * @brief 将逻辑 SPI 从设备注册到 MC-02 的 SPI 和 GPIO 资源。
 */
#include "../spi/platform_spi_blocking.h"
#include "board_spi_stm32.h"
#include "board_gpio_stm32.h"
#include "bsp_spi.h"
#include <stddef.h>

void *PlatformSpiRegisterBlocking(const PlatformSpiEndpoint_t *endpoint, void *owner)
{
    if (endpoint == NULL) return NULL; // 没有端点时不申请设备。
    SPI_HandleTypeDef *bus = BoardSpiResolve(endpoint->bus); // 将逻辑总线映射到本板控制器。
    BoardGpioBinding_t cs;
    if (bus == NULL || !BoardGpioResolve(endpoint->chip_select, &cs))
        return NULL; // 总线或片选缺失时不占用资源。
    const SPI_Init_Config_s config = {
        .spi_handle = bus, // 两个传感器可共享控制器。
        .GPIOx = cs.port, // 片选所在 GPIO 端口。
        .cs_pin = cs.pin, // 本设备独占片选引脚。
        .spi_work_mode = SPI_BLOCK_MODE, // 本接口仅注册同步设备。
        .id = owner, // 保留设备上下文供板级实例使用。
    };
    return SPIRegister(&config); // 复用原有重复和容量检查。
}

uint8_t PlatformSpiUnregister(void *device)
{
    return SPIUnregister(device); // 原有接口检查空指针及总线是否空闲。
}
