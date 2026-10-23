/**
 * @file lcd_internal.h
 * @brief LCD 实现内部使用的共用期限、SPI 发送和恢复入口。
 * @note 不供业务模块包含；一项外层操作建立期限，内部绘图沿用该期限。
 */
#ifndef MC02_LCD_INTERNAL_H
#define MC02_LCD_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include "lcd.h"

LCDStatus_t LCD_InternalBegin(uint32_t budget_ms, uint8_t recovery);
LCDStatus_t LCD_InternalEnd(LCDStatus_t result);
LCDStatus_t LCD_InternalCurrentStatus(void);
LCDStatus_t LCD_InternalCheckDeadline(void);
LCDStatus_t LCD_InternalWriteCommand(uint8_t command);
LCDStatus_t LCD_InternalWriteData8(uint8_t value);
LCDStatus_t LCD_InternalSetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
LCDStatus_t LCD_InternalBeginPixelData(void);
LCDStatus_t LCD_InternalWritePixelData(const uint8_t *data, size_t length);
LCDStatus_t LCD_InternalEndPixelData(void);
LCDStatus_t LCD_InternalStopForRecovery(void);
LCDStatus_t LCD_InternalBeginPanelInit(void);
LCDStatus_t LCD_InternalMarkPanelInitialized(void);
LCDStatus_t LCD_InternalCommitUI(void);

#endif /* MC02_LCD_INTERNAL_H */
