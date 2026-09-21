#ifndef LCD_BUF_H
#define LCD_BUF_H

#include <stdint.h>
#include "lcd_font_cn.h"

void LCD_BufFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_BufShowAscii(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_BufShowCnString(uint16_t x, uint16_t y, const Lcd_CnChar_e *str, uint16_t fc, uint16_t bc);
void LCD_BufShowCnChar(uint16_t x, uint16_t y, Lcd_CnChar_e ch, uint16_t fc, uint16_t bc);
void LCD_BufShowInt(uint16_t x, uint16_t y, int32_t value, uint16_t fc, uint16_t bc, uint8_t sizey);
void LCD_BufShowFloat(uint16_t x, uint16_t y, float value, uint8_t decimals, uint16_t fc, uint16_t bc, uint8_t sizey);

#endif // LCD_BUF_H
