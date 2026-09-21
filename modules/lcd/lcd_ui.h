#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>

#define LCD_PAGE_COUNT 4u

void LCD_UI_Init(void);
void LCD_UI_Recover(void);
void LCD_UI_DrawStatic(uint8_t page);
void LCD_UI_UpdateValues(uint8_t page);
void LCD_UI_SetFrozen(uint8_t frozen);

#endif // LCD_UI_H
