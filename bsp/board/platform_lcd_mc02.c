/** @file platform_lcd_mc02.c
 * @brief 将 LCD 面板传输与引脚绑定到 MC-02 的 SPI1 和 GPIO。
 */
#include "platform_lcd.h"
#include "main.h"
#include "spi.h"
#include "bsp_hal_tick.h"

uint8_t PlatformLcdBusReady(void) { return hspi1.Instance == SPI1; }
uint8_t PlatformLcdInHandler(void) { return __get_IPSR() != 0u; }
uint8_t PlatformLcdAcquireTimebase(void) { return BSP_HALTickTryAcquireBlockingWait(); }
void PlatformLcdReleaseTimebase(void) { BSP_HALTickReleaseBlockingWait(); }
uint32_t PlatformLcdNowMs(void) { return HAL_GetTick(); }
void PlatformLcdDelayMs(uint32_t milliseconds) { HAL_Delay(milliseconds); }
void PlatformLcdReset(uint8_t high) { HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void PlatformLcdDataMode(uint8_t high) { HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void PlatformLcdChipSelect(uint8_t high) { HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void PlatformLcdBacklight(uint8_t high) { HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET); }

PlatformLcdTransferStatus_t PlatformLcdTransmit(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, timeout_ms);
    if (result == HAL_OK) return PLATFORM_LCD_TRANSFER_OK;
    if (result == HAL_BUSY) return PLATFORM_LCD_TRANSFER_BUSY;
    if (result == HAL_TIMEOUT) return PLATFORM_LCD_TRANSFER_TIMEOUT;
    return PLATFORM_LCD_TRANSFER_ERROR;
}

uint8_t PlatformLcdStop(void)
{
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) return 1u; // 空闲时无需中止。
    if (HAL_SPI_Abort(&hspi1) != HAL_OK) return 0u; // 中止失败时不能复用总线。
    return (uint8_t)(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY); // 再次确认空闲。
}
