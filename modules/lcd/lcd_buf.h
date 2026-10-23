/**
 * @file lcd_buf.h
 * @brief LCD 单行像素缓冲绘图与安全数值文本转换接口。
 * @note 先成功调用 LCD_Init()；所有绘图由同一普通任务串行执行，不保留输入地址。
 *       坐标原点在绘图区左上角，末端坐标包含在矩形内，颜色使用 RGB565。
 */
#ifndef MC02_LCD_BUF_H
#define MC02_LCD_BUF_H

#include <stddef.h>
#include <stdint.h>
#include "lcd.h"
#include "lcd_font_cn.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 清黑整块面板，包括绘图区外的圆角留白；顶层期限 250ms。 */
LCDStatus_t LCD_BufClearPanel(void);

/** @brief 整屏清理最近耗时，单位 us。 */
extern volatile uint32_t lcd_prof_clear_us;

/**
 * @brief 填充包含两端坐标的矩形。
 * @param[in] x1,y1 左上角像素坐标，范围为绘图区内。
 * @param[in] x2,y2 右下角像素坐标，须满足 x1<=x2、y1<=y2。
 * @param[in] color RGB565 颜色值。
 * @return LCD_OK 或参数、范围、初始化、期限和 SPI 错误；失败后不保证面板已写入的像素可回滚。
 */
LCDStatus_t LCD_BufFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief 按指定字高覆盖绘制一段 ASCII 文本，不换行、不裁剪。
 * @param[in] x,y 文本左上角像素坐标。
 * @param[in] str 字节数组；length 为 0 时可为 NULL，其他情况须有 length 个可读字节。
 * @param[in] length 文本字节数，不包含 C 字符串结束符；全部字节须在 0x20～0x7E。
 * @param[in] fc,bc RGB565 前景色和背景色。
 * @param[in] sizey 字高，仅允许 12 或 16 像素。
 * @return LCD_OK 或参数、文字、范围、初始化、期限和 SPI 错误；拒绝时不发送。
 */
LCDStatus_t LCD_BufShowAscii(uint16_t x, uint16_t y, const char *str, size_t length,
                             uint16_t fc, uint16_t bc, uint8_t sizey);

/**
 * @brief 绘制一组 16×16 中文字形，不寻找结束标记，也不换行。
 * @param[in] x,y 第一个字左上角像素坐标。
 * @param[in] glyphs 字形编号数组；count 为 0 时可为 NULL，其他情况须有 count 项可读。
 * @param[in] count 字形数，不包含 LCD_CN_COUNT 结束项；每项须小于 LCD_CN_COUNT。
 * @param[in] fc,bc RGB565 前景色和背景色。
 * @return LCD_OK 或参数、字形、范围、初始化、期限和 SPI 错误；拒绝时不发送。
 */
LCDStatus_t LCD_BufShowCnString(uint16_t x, uint16_t y, const Lcd_CnChar_e *glyphs, size_t count,
                                uint16_t fc, uint16_t bc);

/**
 * @brief 绘制一个 16×16 中文字形。
 * @param[in] x,y 字形左上角像素坐标。
 * @param[in] ch 字形编号，范围为 0～LCD_CN_COUNT-1。
 * @param[in] fc,bc RGB565 前景色和背景色。
 * @return LCD_OK 或参数、字形、范围、初始化、期限和 SPI 错误。
 */
LCDStatus_t LCD_BufShowCnChar(uint16_t x, uint16_t y, Lcd_CnChar_e ch, uint16_t fc, uint16_t bc);

/**
 * @brief 将 int32_t 写成十进制文本并绘制。
 * @param[in] x,y 文本左上角像素坐标。
 * @param[in] value 完整 int32_t 范围内的整数。
 * @param[in] fc,bc RGB565 前景色和背景色。
 * @param[in] sizey 字高，仅允许 12 或 16 像素。
 * @return LCD_OK 或参数、范围、初始化、期限和 SPI 错误。
 */
LCDStatus_t LCD_BufShowInt(uint16_t x, uint16_t y, int32_t value,
                           uint16_t fc, uint16_t bc, uint8_t sizey);

/**
 * @brief 把有限浮点数格式化成固定小数位的十进制文本。
 * @param[in] value 有限输入；缩放并舍入后的结果须在 INT32_MIN～INT32_MAX 内。
 * @param[in] decimals 小数位数，范围 0～3；采用最近值、半数远离零。
 * @param[out] output 至少可写入结果字符和一个结束符的缓冲；失败时内容不变。
 * @param[in] capacity output 的字节容量。
 * @param[out] length 非空长度输出，不含结束符；失败时保留原值。
 * @return LCD_OK、LCD_BAD_ARGUMENT、LCD_NONFINITE、LCD_OUT_OF_RANGE 或 LCD_BAD_TEXT。
 */
LCDStatus_t LCD_BufFormatFloat(float value, uint8_t decimals, char *output,
                               size_t capacity, size_t *length);

/**
 * @brief 格式化有限浮点数并按 ASCII 规则绘制。
 * @param[in] x,y 文本左上角像素坐标。
 * @param[in] value 有限输入；缩放并舍入后的结果须在 INT32_MIN～INT32_MAX 内。
 * @param[in] decimals 小数位数，范围 0～3；半数远离零。
 * @param[in] fc,bc RGB565 前景色和背景色。
 * @param[in] sizey 字高，仅允许 12 或 16 像素。
 * @return LCD_OK 或数值、范围、初始化、期限和 SPI 错误；拒绝时不发送。
 */
LCDStatus_t LCD_BufShowFloat(uint16_t x, uint16_t y, float value, uint8_t decimals,
                             uint16_t fc, uint16_t bc, uint8_t sizey);

#ifdef __cplusplus
}
#endif

#endif /* MC02_LCD_BUF_H */
