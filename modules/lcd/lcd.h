/**
 * @file lcd.h
 * @brief LCD 初始化、直接绘图和文字显示接口，默认通过 SPI1 发送数据。
 *
 * 先配置 GPIO、SPI 和 HAL 毫秒计时，再调用 LCD_Init()。当前界面主要通过
 * lcd_buf.c 成批发送像素，本文件也保留逐点、逐字绘制入口。
 * 坐标单位均为像素，原点在绘图区左上角：x 向右、y 向下，范围分别为
 * 0～LCD_W-1、0～LCD_H-1。颜色均为 RGB565：红色 5 位、绿色 6 位、蓝色 5 位。
 * 接口不检查越界或空指针，也不返回 SPI 发送结果；调用方须保证整块绘制区域
 * 在屏内、数据有效，并避免多个任务同时操作 LCD。
 */
#ifndef __LCD_H
#define __LCD_H


#include "main.h"
#include "spi.h"


#define USE_ANALOG_SPI 0	// 0：硬件 SPI1；1：用 GPIO 逐位发送。
#define USE_HORIZONTAL 2  // 0、1：竖屏的两个方向；2、3：横屏的两个方向。


// 绘图区四周留白，避开屏幕圆角；单位为像素，数值越大，绘图区越小。
#define LCD_MARGIN_X 14
#define LCD_MARGIN_Y 14

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W (240 - 2 * LCD_MARGIN_X)
#define LCD_H (280 - 2 * LCD_MARGIN_Y)

#else
#define LCD_W (280 - 2 * LCD_MARGIN_X)
#define LCD_H (240 - 2 * LCD_MARGIN_Y)
#endif

// 整块面板在控制器中的坐标和尺寸，已含 20 像素偏移，不含四周留白。
// LCD_BufClearPanel() 直接使用这些坐标；不要传给会再次加偏移的 LCD_Address_Set()。
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_PANEL_X0 0
#define LCD_PANEL_Y0 20
#define LCD_PANEL_W  240
#define LCD_PANEL_H  280
#else
#define LCD_PANEL_X0 20
#define LCD_PANEL_Y0 0
#define LCD_PANEL_W  280
#define LCD_PANEL_H  240
#endif

 
// LCD 引脚：Clr 输出低电平，Set 输出高电平。
#if USE_ANALOG_SPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_RESET)
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port,LCD_SCK_Pin, GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_RESET)
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_SDA_Pin, GPIO_PIN_SET)
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_RESET)// 复位脚，低电平复位。
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_RESET)// 低电平写命令，高电平写数据。
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin, GPIO_PIN_SET)
 		     
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_RESET)// 片选脚，低电平选中 LCD。
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin, GPIO_PIN_SET)

// 背光只有开关控制；保留的参数 x 未使用，不能通过它调节亮度。
#define LCD_BLK_Clr(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_RESET)
#define LCD_BLK_Set(x)  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin, GPIO_PIN_SET)

/**
 * @brief 复位 LCD、打开背光并设置显示方向和颜色格式，不执行清屏。
 * @note 调用前须完成外设配置，且 HAL_Delay() 能正常计时；四次延时合计 420 ms。
 *       再次调用仍会复位和等待，调用后由上层清屏、重画界面。
 */
void LCD_Init(void);

/**
 * @brief 发送一个字节，每次发送前拉低片选，结束后拉高片选。
 * @param[in] dat 待发送的 8 位数据，高位先发。
 * @note 不改变命令/数据引脚。硬件 SPI 的等待参数为 65535 ms，发送状态未返回。
 */
void LCD_Writ_Bus(uint8_t dat);

/**
 * @brief 发送一个数据字节，不改变命令/数据引脚。
 * @param[in] dat 待发送的 8 位数据；调用前须处于写数据状态。
 */
void LCD_WR_DATA8(uint8_t dat);

/**
 * @brief 发送一个 16 位数据，高字节先发；一个 RGB565 像素使用此格式。
 * @param[in] dat 待发送的数据；调用前须处于写数据状态。
 */
void LCD_WR_DATA(uint16_t dat);

/**
 * @brief 切到命令状态发送命令，随后恢复写数据状态。
 * @param[in] dat LCD 命令编号。
 */
void LCD_WR_REG(uint8_t dat);

/**
 * @brief 设置接下来写像素的矩形区域，包含起点和终点。
 * @param[in] x1,y1 左上角坐标，单位为像素。
 * @param[in] x2,y2 右下角坐标，须满足 x1 <= x2 < LCD_W、y1 <= y2 < LCD_H。
 * @note 内部按屏幕方向加上面板偏移和四周留白，不要预先给坐标加偏移。
 */
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

/**
 * @brief 用同一种颜色填充矩形，不绘制末端 xend 所在列和 yend 所在行。
 * @param[in] xsta,ysta 左上角坐标，单位为像素。
 * @param[in] xend,yend 末端坐标；须满足 xsta < xend <= LCD_W、ysta < yend <= LCD_H。
 * @param[in] color RGB565 填充颜色。
 * @note 与 LCD_BufFill() 包含末端坐标的用法不同；本函数不检查空区域。
 */
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);

/**
 * @brief 绘制一个像素。
 * @param[in] x,y 绘图区内的坐标，单位为像素。
 * @param[in] color RGB565 颜色。
 */
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);

/**
 * @brief 从起点朝终点逐点画线，沿用当前步进方法，不额外补画终点。
 * @param[in] x1,y1 起点坐标，单位为像素。
 * @param[in] x2,y2 终点坐标，单位为像素；整条线须在绘图区内。
 * @param[in] color RGB565 线条颜色。
 */
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color);

/**
 * @brief 调用 LCD_DrawLine() 绘制矩形四边，不填充内部。
 * @param[in] x1,y1 一个角的坐标，单位为像素。
 * @param[in] x2,y2 对角坐标，单位为像素；整个矩形须在绘图区内。
 * @param[in] color RGB565 边框颜色。
 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);

/**
 * @brief 绘制圆周，不填充内部。
 * @param[in] x0,y0 圆心坐标，单位为像素。
 * @param[in] r 半径，单位为像素，取值 0～255；整个圆须在绘图区内。
 * @param[in] color RGB565 颜色。
 */
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color);

/**
 * @brief 每次取两个字节查找汉字，向右排列，遇到 0 结束，不自动换行。
 * @param[in] x,y 第一个字的左上角坐标，单位为像素。
 * @param[in] s 两字节一组、以 0 结尾的字库索引串，指针不能为 NULL。
 * @param[in] fc,bc RGB565 字色和背景色。
 * @param[in] sizey 字的宽、高，单位为像素，支持 12、16、24、32，其他值停止绘制。
 * @param[in] mode 0：字和背景都画；非 0：只画字形，保留原背景，bc 不使用。
 * @note 只比较字库 Index 的前两个字节，不负责 UTF-8 解码；找不到字时不绘制，
 *       仍向右移动一个字宽。调用方须保证整串文字在绘图区内。
 */
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/**
 * @brief 从 12×12 字库中查找并绘制一个汉字。
 * @param[in] x,y 字的左上角坐标，单位为像素，整个字须在绘图区内。
 * @param[in] s 至少可读两个字节的索引，指针不能为 NULL；只读取、不改写。
 * @param[in] fc,bc RGB565 字色和背景色。
 * @param[in] sizey 须等于函数名中的字高，单位为像素；函数不会检查是否匹配。
 * @param[in] mode 0：同时画字和背景；非 0：只画字形，bc 不使用。
 * @note 只匹配 Index 前两个字节，找不到就不绘制；找到后仍会继续检查后续字库项。
 */
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/** @brief 从 16×16 字库中绘制一个汉字；sizey 须为 16。
 *  @copydetails LCD_ShowChinese12x12
 */
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/** @brief 从 24×24 字库中绘制一个汉字；sizey 须为 24。
 *  @copydetails LCD_ShowChinese12x12
 */
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/** @brief 从 32×32 字库中绘制一个汉字；sizey 须为 32。
 *  @copydetails LCD_ShowChinese12x12
 */
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/**
 * @brief 绘制一个 ASCII 字符，宽度为字高的一半。
 * @param[in] x,y 左上角坐标，单位为像素，整个字符须在绘图区内。
 * @param[in] num ASCII 编号，须为 32～126，即空格到 ~；未检查编号范围。
 * @param[in] fc,bc RGB565 字色和背景色。
 * @param[in] sizey 字高，单位为像素，支持 12、16、24、32。
 * @param[in] mode 0：同时画字和背景；非 0：只画字形，bc 不使用。
 * @note 不支持的字高不会绘制字符，但函数已先设置绘制区域。
 */
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/**
 * @brief 向右绘制以 0 结尾的 ASCII 字符串，不处理换行或汉字。
 * @param[in] x,y 第一个字符的左上角坐标，单位为像素。
 * @param[in] p 字符串，指针不能为 NULL；结尾 0 之前的字符须在 ASCII 32～126 之间。
 * @param[in] fc,bc RGB565 字色和背景色。
 * @param[in] sizey 字高，单位为像素，支持 12、16、24、32；每字前进 sizey/2 像素。
 * @param[in] mode 0：同时画字和背景；非 0：只画字形，bc 不使用。
 * @note 调用方须保证整串文字在绘图区内。
 */
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

/**
 * @brief 重复相乘，计算 m 的 n 次方，供数字显示拆位使用。
 * @param[in] m 底数。
 * @param[in] n 指数，为 0 时返回 1。
 * @return 计算结果；超出 uint32_t 范围时只保留低 32 位，不报告错误。
 */
uint32_t mypow(uint8_t m,uint8_t n);

/**
 * @brief 按十进制显示整数，前面的 0 用空格代替，数值为 0 时仍显示末位 0。
 * @param[in] x,y 数字区域左上角坐标，单位为像素。
 * @param[in] num 待显示的无符号整数。
 * @param[in] len 显示位数，使用 1～10；为 0 时不绘制，位数不足时只显示低位。
 * @param[in] fc,bc RGB565 字色和背景色，绘制时覆盖背景。
 * @param[in] sizey 字高，单位为像素，支持 12、16、24、32。
 * @note 函数不检查 len；整个 len*sizey/2 像素宽的区域须在绘图区内。
 */
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey);

/**
 * @brief 显示浮点数，先画负号或空格，再画数字；不做四舍五入。
 * @param[in] x,y 符号位置的左上角坐标，单位为像素。
 * @param[in] num 待显示数值；num*10^decimal 须在 -32767～32767 之间。
 * @param[in] len 使用 0～9；当前实现的整数位数为 len+1-decimal，含前面的 0。
 * @param[in] decimal 小数位数，须为 0～len；为 0 时不画小数点。
 * @param[in] fc,bc RGB565 字色和背景色，绘制前会清理数字区背景。
 * @param[in] sizey 字高，单位为像素，支持 12、16、24、32。
 * @note 不检查输入，不接受 NaN 或无穷大；位数不足时只显示低位。
 *       背景清理比字高多一行，调用方须为符号、数字、小数点和清理区域留出空间。
 */
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);

/**
 * @brief 显示非负浮点数；先空出一个字符位置，该位置不绘制、不清理。
 * @param[in] x,y 预留位置的左上角坐标，单位为像素，数字从其右侧开始。
 * @param[in] num 待显示的非负数；num*10^decimal 须在 0～32767 之间，不做四舍五入。
 * @param[in] len 使用 0～9；当前实现的整数位数为 len+1-decimal，含前面的 0。
 * @param[in] decimal 小数位数，须为 0～len；为 0 时不画小数点。
 * @param[in] fc,bc RGB565 字色和背景色，绘制前会清理数字区背景。
 * @param[in] sizey 字高，单位为像素，支持 12、16、24、32。
 * @note 不检查输入，不接受 NaN 或无穷大；位数不足时只显示低位。
 *       背景清理比字高多一行，整个绘制和清理区域须在绘图区内。
 */
void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint8_t decimal, uint16_t fc, uint16_t bc, uint8_t sizey);

/**
 * @brief 按数组顺序发送 RGB565 图片，不解析图片文件格式。
 * @param[in] x,y 图片左上角坐标，单位为像素，整张图片须在绘图区内。
 * @param[in] length 水平方向的像素数，须大于 0。
 * @param[in] width 垂直方向的像素数，须大于 0。
 * @param[in] pic 像素数组，指针不能为 NULL，至少有 2*length*width 字节，高字节在前。
 * @note 在本次调用中读取数组并发送，返回后不保留数组地址；不检查数组长度。
 */
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);


// 常用 RGB565 颜色值；保留原有名称和数值，绘制时每像素发送两个字节。

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
#define BROWN 			     0XBC40
#define BRRED 			     0XFC07
#define GRAY  			     0X8430
#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458
#define LIGHTGREEN     	 0X841F
#define LGRAY 			     0XC618
#define LGRAYBLUE        0XA651
#define LBBLUE           0X2B12
#endif





