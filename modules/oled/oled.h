/**
 * @file oled.h
 * @author your name (you@domain.com)
* @brief 声明 OLED 屏幕的初始化、像素绘制、文字输出和整屏刷新接口。
* 绘图函数先改写 128×8 页缓冲区；调用 OLED_refresh_gram 后才把缓冲区发到屏幕。
 * @version 0.1
 * @date 2023-02-14
 * @todo 请重构show string/init/clean/update buffer等的实现
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef OLED_H
#define OLED_H
#include <stdint.h>
#include "format_check.h"

// OLED 的 I2C 从机地址。
#define OLED_I2C_ADDRESS 0x78

// 屏幕分辨率为128×64像素；缓冲区按128列、8页保存，每页对应8行像素。
#define MAX_COLUMN 128
#define MAX_ROW 64

#define X_WIDTH MAX_COLUMN
#define Y_WIDTH MAX_ROW

#define OLED_CMD 0x00
#define OLED_DATA 0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12

typedef enum
{
  PEN_CLEAR = 0x00,
  PEN_WRITE = 0x01,
  PEN_INVERSION = 0x02,
} pen_typedef;

/** @brief 发送 OLED 控制器初始化指令；不清除显存缓冲区。 */
extern void OLED_init(void);

/** @brief 打开 OLED 面板显示。 */
extern void OLED_display_on(void);

/** @brief 关闭 OLED 面板显示，显存缓冲区保留。 */
extern void OLED_display_off(void);

/** @brief 按指定方式改写全部128×8个缓冲字节，不立即刷新屏幕。
* @param pen PEN_CLEAR 清零，PEN_WRITE 全置1，其他值按位取反。
 */
extern void OLED_operate_gram(pen_typedef pen);

/** @brief 设置面板写入位置。
* @param x 列地址，范围0～127，单位像素列。
* @param y 页地址，范围0～7，每页对应8行像素。
 */
extern void OLED_set_pos(uint8_t x, uint8_t y);

/** @brief 改写缓冲区中的一个像素；坐标超出屏幕时直接返回。
* @param x 横坐标，范围0～127像素。
* @param y 纵坐标，范围0～63像素。
* @param pen PEN_CLEAR 清除，PEN_WRITE 点亮，PEN_INVERSION 反转当前像素。
 */
extern void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen);

/** @brief 在缓冲区中连接两个像素点，不立即刷新屏幕。
* @param x1 起点横坐标，范围0～127像素。
* @param y1 起点纵坐标，范围0～63像素。
* @param x2 终点横坐标，范围0～127像素。
* @param y2 终点纵坐标，范围0～63像素。
* @param pen 每个线段像素使用的绘制方式。
 */
extern void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);

/** @brief 在缓冲区中绘制一个6×12字符；刷新屏幕需另调 OLED_refresh_gram。
* @param row 字符行，范围0～4。
* @param col 字符列，范围0～20。
* @param chr 可显示的 ASCII 字符，范围空格到波浪号（0x20～0x7e）。
 */
extern void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr);

/** @brief 按字符逐个绘制以 '\0' 结束的字符串；到列20后从下一行第0列继续。
* @param row 起始行，范围0～4。
* @param col 起始列，范围0～20。
* @param chr 可读的零结尾字符串，字符应在空格到波浪号之间。
 */
extern void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr);

/** @brief 格式化文字并绘制在同一行的缓冲区中；不会自动刷新屏幕。
* @param[in] row 起始行，范围0～4，超范围直接返回。
* @param[in] col 起始列，范围0～20，超范围直接返回。
* @param[in] fmt 非空可读printf格式字符串，后续参数按格式提供，所引用对象保持可读至调用结束。
* @note 当前行剩余字符以外的内容会被丢弃；格式化结果须连结束标记不超过128字节。
*       编译属性只检查已知格式的参数，当前实现仍要求调用方保证长度；运行时容量修复由MISC-06跟踪。
 */
extern void OLED_printf(uint8_t row, uint8_t col, const char *fmt, ...) MC02_PRINTF_FORMAT(3, 4);


/** @brief 按页发送整个128×64显存缓冲区到 OLED 面板。 */
extern void OLED_refresh_gram(void);


/** @brief 清空缓冲区、绘制 RM 标志并刷新整屏。 */
extern void OLED_LOGO(void);
#endif
