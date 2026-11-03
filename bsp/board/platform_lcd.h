/** @file platform_lcd.h
 * @brief LCD 面板使用的阻塞传输及板级引脚接口。
 */
#ifndef MC02_PLATFORM_LCD_H
#define MC02_PLATFORM_LCD_H
#include <stdint.h>

/** @brief 一次面板传输的结果。 */
typedef enum {
    PLATFORM_LCD_TRANSFER_OK, /**< 所有字节已由平台传输完成。 */
    PLATFORM_LCD_TRANSFER_BUSY, /**< 传输资源正忙。 */
    PLATFORM_LCD_TRANSFER_TIMEOUT, /**< 本次传输超过允许时间。 */
    PLATFORM_LCD_TRANSFER_ERROR /**< 其他传输错误。 */
} PlatformLcdTransferStatus_t;

/** @brief 查询面板总线是否已初始化；不代表面板在线。 */
uint8_t PlatformLcdBusReady(void);
/** @brief 查询当前是否处于中断或异常处理。 */
uint8_t PlatformLcdInHandler(void);
/** @brief 为阻塞等待申请可用时基；成功后必须配对释放。 */
uint8_t PlatformLcdAcquireTimebase(void);
/** @brief 释放一次成功申请的时基。 */
void PlatformLcdReleaseTimebase(void);
/** @brief 读取既有本机毫秒计数，允许 32 位回绕。 */
uint32_t PlatformLcdNowMs(void);
/** @brief 阻塞等待指定毫秒；调用前须持有时基。 */
void PlatformLcdDelayMs(uint32_t milliseconds);
/** @brief 设置复位引脚电平；high 为非零时释放复位。 */
void PlatformLcdReset(uint8_t high);
/** @brief 设置数据选择电平；high 为非零时选择数据。 */
void PlatformLcdDataMode(uint8_t high);
/** @brief 设置片选电平；high 为非零时释放面板。 */
void PlatformLcdChipSelect(uint8_t high);
/** @brief 设置背光电平；high 为非零时打开。 */
void PlatformLcdBacklight(uint8_t high);
/** @brief 阻塞发送只读字节；调用前须持有时基。
 * @param[in] data 可读字节，在调用返回前保持有效且非空。
 * @param[in] length 字节数，范围 1～65535。
 * @param[in] timeout_ms 此次传输的等待上限，单位毫秒，必须非零。
 * @return 传输结果；成功不表示面板已显示。
 */
PlatformLcdTransferStatus_t PlatformLcdTransmit(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
/** @brief 停止总线传输并确认总线空闲。
 * @return 非零表示已确认空闲；零表示无法确认，调用者须保持隔离。
 */
uint8_t PlatformLcdStop(void);
#endif
