#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>

#define LCD_PAGE_COUNT 4u

void LCD_UI_Init(void);
void LCD_UI_Recover(void);
void LCD_UI_DrawStatic(uint8_t page);
void LCD_UI_UpdateValues(uint8_t page);
void LCD_UI_SetFrozen(uint8_t frozen);
/* 按需标定提示: 0=清除并重画整页, 1=标定中, 2=成功, 3=失败 */
void LCD_UI_ShowCalibMsg(uint8_t msg);

#endif // LCD_UI_H
