/** @file platform_spi_blocking.h
 * @brief 声明同步 SPI 全双工传输，供传感器在任务内读取寄存器。
 */
#ifndef MC02_PLATFORM_SPI_BLOCKING_H
#define MC02_PLATFORM_SPI_BLOCKING_H
#include <stdint.h>

/** @brief 同步传输结果；数值保留既有初始化诊断含义。 */
typedef enum {
    PLATFORM_SPI_OK = 0, // 收发完成，可以读取全部接收字节。
    PLATFORM_SPI_ERROR = 1, // 参数、模式或设备错误。
    PLATFORM_SPI_BUSY = 2, // 当前不能取得总线或调用上下文不允许。
    PLATFORM_SPI_TIMEOUT = 3 // 等待总线或传输超过期限。
} PlatformSpiStatus_t;

/** @brief 同步发送和接收同样数量的字节。
 * @param[in,out] device 平台注册的设备，必须保持有效直到调用返回。
 * @param[out] rx 至少 length 字节；失败时内容不可使用。
 * @param[in] tx 至少 length 字节，不与 rx 重叠。
 * @param[in] length 字节数，范围 1～255。
 * @return 传输完成或失败原因；非阻塞设备会被拒绝。
 * @note 调用返回后缓冲可复用；调用环境须满足板级 SPI 的任务上下文限制。
 */
PlatformSpiStatus_t PlatformSpiTransferBlocking(void *device, uint8_t *rx,
                                                uint8_t *tx, uint8_t length);
#endif
