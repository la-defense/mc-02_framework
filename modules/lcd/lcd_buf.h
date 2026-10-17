/**
* @file lcd_buf.h
* @brief 先在 RAM 中准备一行像素，再成批发送到 LCD，减少逐像素发送的开销。
* @note 先调用 LCD_Init()；这些接口共用一行数组，应由同一个任务依次调用。
*       坐标以绘图区左上角为原点，x 向右、y 向下，单位为像素；颜色使用 RGB565。
*       接口同步发送，不保留传入地址；发送失败会计数并中止本次 SPI 传输，随后继续处理。
*/
#ifndef LCD_BUF_H
#define LCD_BUF_H

#include <stdint.h>
#include "lcd_font_cn.h"

/** @brief 将整块面板刷黑，包含绘图区四周留白，供初始化和重连后清理旧画面。 */
void LCD_BufClearPanel(void);
extern volatile uint32_t lcd_prof_clear_us; /* 整屏清屏最近耗时(us), 调试用 */
/**
* @brief 填充矩形，包含起点和末端；坐标越界或起点大于末端时直接返回。
* @param[in] x1,y1 左上角坐标，单位为像素。
* @param[in] x2,y2 右下角坐标，须小于 LCD_W、LCD_H。
* @param[in] color RGB565 填充颜色。
*/
void LCD_BufFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
/**
* @brief 成行绘制 ASCII 字符串，覆盖背景，不自动换行。
* @param[in] x,y 左上角坐标，单位为像素。
* @param[in] str 以 0 结尾的短字符串；NULL 或空串不绘制，32～126 之外的字符画成空格。
* @param[in] fc,bc RGB565 字色和背景色。
* @param[in] sizey 字高，仅支持 12、16 像素，字宽为字高的一半。
* @note 字号不支持或整串文字放不进绘图区时直接返回；调用方须限制字符串长度，
*       内部长度按 uint16_t 保存，不适合传入很长的文本。
*/
void LCD_BufShowAscii(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc, uint8_t sizey);
/**
* @brief 绘制 16×16 中文字形序列，覆盖背景，区域越界时不绘制。
* @param[in] x,y 左上角坐标，单位为像素。
* @param[in] str 字形枚举数组，以 LCD_CN_COUNT 结束，NULL 或首项即结束时不绘制。
* @param[in] fc,bc RGB565 字色和背景色。
* @note 每项须是有效字形编号；内部最多计 32 个字，但判断时会先读 str[len]，
*       调用方仍须准备可读的结束标记，不能依靠此上限保护短数组。
*/
void LCD_BufShowCnString(uint16_t x, uint16_t y, const Lcd_CnChar_e *str, uint16_t fc, uint16_t bc);
/**
* @brief 绘制一个 16×16 中文字形，覆盖背景；区域越界时不绘制。
* @param[in] x,y 左上角坐标，单位为像素。
* @param[in] ch 有效字形编号；LCD_CN_COUNT 表示不绘制，其他越界编号未检查。
* @param[in] fc,bc RGB565 字色和背景色。
*/
void LCD_BufShowCnChar(uint16_t x, uint16_t y, Lcd_CnChar_e ch, uint16_t fc, uint16_t bc);
/**
* @brief 将有符号整数转成十进制文本，再按 ASCII 字符串绘制。
* @param[in] x,y 左上角坐标，单位为像素。
* @param[in] value 待显示整数。
* @param[in] fc,bc RGB565 字色和背景色。
* @param[in] sizey 字高，仅支持 12、16 像素；整串放不下时不绘制。
*/
void LCD_BufShowInt(uint16_t x, uint16_t y, int32_t value, uint16_t fc, uint16_t bc, uint8_t sizey);
/**
* @brief 将浮点数四舍五入后转成文本，再按 ASCII 字符串绘制。
* @param[in] x,y 左上角坐标，单位为像素。
* @param[in] value 待显示数值，须排除 NaN、无穷大；放大并加减 0.5 后须在 -2147483647～2147483647 之间。
* @param[in] decimals 小数位数，超过 3 时按 3 处理；先乘以 10 的对应次方再取整数。
* @param[in] fc,bc RGB565 字色和背景色。
* @param[in] sizey 字高，仅支持 12、16 像素；整串放不下时不绘制。
* @note 不检查数值范围；舍入后为 0 时不显示负号。
*/
void LCD_BufShowFloat(uint16_t x, uint16_t y, float value, uint8_t decimals, uint16_t fc, uint16_t bc, uint8_t sizey);

#endif // LCD_BUF_H
