#ifndef __LCD_H
#define __LCD_H


#include "main.h"
#include "spi.h"


#define USE_ANALOG_SPI 0	// 鍙€夌殑杞欢SPI鎴栬€呯‖浠禨PI		0: 纭欢SPI  1: 杞欢SPI
#define USE_HORIZONTAL 2  // 璁剧疆妯睆鎴栬€呯珫灞忔樉绀?0鎴?涓虹珫灞?2鎴?涓烘í灞?

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif

 
//-----------------LCD寮曡剼瀹氫箟---------------- 
#if USE_ANALOG_SPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)

#define LCD_BLK_Clr(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_RESET)//BLK TIM1->CCR1=x//
#define LCD_BLK_Set(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_SET)//TIM1->CCR1=x//

void LCD_Init(void); 						//LCD鍒濆鍖?
void LCD_Writ_Bus(uint8_t dat);	// 妯℃嫙SPI鏃跺簭
void LCD_WR_DATA8(uint8_t dat);	// 鍐欏叆涓€涓瓧鑺?
void LCD_WR_DATA(uint16_t dat);	// 鍐欏叆涓や釜瀛楄妭
void LCD_WR_REG(uint8_t dat);		// 鍐欏叆涓€涓寚浠?
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2); // 璁剧疆鍧愭爣鍑芥暟

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color); // 鎸囧畾鍖哄煙濉厖棰滆壊
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color); // 鍦ㄦ寚瀹氫綅缃敾涓€涓偣
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color); // 鍦ㄦ寚瀹氫綅缃敾涓€鏉＄嚎
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color); // 鍦ㄦ寚瀹氫綅缃敾涓€涓煩褰?
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color); // 鍦ㄦ寚瀹氫綅缃敾涓€涓渾

void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); 			// 鏄剧ず姹夊瓧涓?
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // 鏄剧ず鍗曚釜12x12姹夊瓧
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // 鏄剧ず鍗曚釜16x16姹夊瓧
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // 鏄剧ず鍗曚釜24x24姹夊瓧
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // 鏄剧ず鍗曚釜32x32姹夊瓧

void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);				// 鏄剧ず涓€涓瓧绗?
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode); // 鏄剧ず瀛楃涓?
uint32_t mypow(uint8_t m,uint8_t n); // 姹傚箓
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey); // 鏄剧ず鏁存暟鍙橀噺
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);	// 鏄剧ず甯︾鍙风殑娴偣鏁?
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);	// 鏄剧ず姝ｇ殑娴偣鏁?
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]); //鏄剧ず鍥剧墖


//鐢荤瑪棰滆壊

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //妫曡壊
#define BRRED 			     0XFC07 //妫曠孩鑹?
#define GRAY  			     0X8430 //鐏拌壊
#define DARKBLUE      	 0X01CF	//娣辫摑鑹?
#define LIGHTBLUE      	 0X7D7C	//娴呰摑鑹?
#define GRAYBLUE       	 0X5458 //鐏拌摑鑹?
#define LIGHTGREEN     	 0X841F //娴呯豢鑹?
#define LGRAY 			     0XC618 //娴呯伆鑹?PANNEL),绐椾綋鑳屾櫙鑹?
#define LGRAYBLUE        0XA651 //娴呯伆钃濊壊(涓棿灞傞鑹?
#define LBBLUE           0X2B12 //娴呮钃濊壊(閫夋嫨鏉＄洰鐨勫弽鑹?
#endif





