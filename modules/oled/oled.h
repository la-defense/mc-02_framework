/**
 * @file oled.h
 * @author your name (you@domain.com)
* @brief 声明 OLED 屏幕的初始化、像素绘制、文字输出和整屏刷新接口。
* 绘图函数先改写 128×8 页缓冲区；调用 OLED_refresh_gram 后才把缓冲区发到屏幕。
 * @version 0.1
 * @date 2023-02-14
 * @todo 请重构show string/init/clean/update buffer等的实现
 * @note 当前未接入业务。所有入口由同一调用者串行使用，不从中断调用；
 *       先配置hi2c2再初始化及刷新。绘图只改RAM，void返回不表示面板通信成功。
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

/** @brief 配置hi2c2后发送面板初始化指令；不清除RAM缓冲区。
 * @return 无返回值，I2C错误不通过本接口报告。
 */
extern void OLED_init(void);

/** @brief 打开已经初始化的 OLED 面板显示。
 * @return 无返回值，I2C错误不通过本接口报告。
 */
extern void OLED_display_on(void);

/** @brief 关闭 OLED 面板显示，显存缓冲区保留。
 * @return 无返回值，I2C错误不通过本接口报告。
 */
extern void OLED_display_off(void);

/** @brief 按指定方式改写全部128×8个缓冲字节，不立即刷新屏幕。
* @param[in] pen PEN_CLEAR清零、PEN_WRITE全置1、PEN_INVERSION逐位反转。
* @return 无返回值；非法画笔不修改缓冲区。
 */
extern void OLED_operate_gram(pen_typedef pen);

/** @brief 设置面板写入位置。
* @param[in] x 列地址，范围0～127，单位像素列。
* @param[in] y 页地址，范围0～7，每页对应8行像素。
* @return 无返回值；非法坐标不发送命令，I2C错误不通过本接口报告。
 */
extern void OLED_set_pos(uint8_t x, uint8_t y);

/** @brief 改写缓冲区中的一个像素；坐标超出屏幕时直接返回。
* @param[in] x 横坐标，范围0～127像素。
* @param[in] y 纵坐标，范围0～63像素。
* @param[in] pen PEN_CLEAR清除、PEN_WRITE点亮、PEN_INVERSION反转当前像素。
* @return 无返回值；非法坐标或画笔不修改缓冲区。
 */
extern void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen);

/** @brief 在缓冲区中连接两个像素点，不立即刷新屏幕。
* @param[in] x1 起点横坐标，范围0～127像素。
* @param[in] y1 起点纵坐标，范围0～63像素。
* @param[in] x2 终点横坐标，范围0～127像素。
* @param[in] y2 终点纵坐标，范围0～63像素。
* @param[in] pen PEN_CLEAR、PEN_WRITE或PEN_INVERSION。
* @return 无返回值；非法端点或画笔整条拒绝，原缓冲区不变。
* @note 包含两个端点，交换端点结果一致；不对屏外线段进行裁剪。
 */
extern void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);

/** @brief 在缓冲区中绘制一个6×12字符；刷新屏幕需另调 OLED_refresh_gram。
* @param[in] row 字符行，范围0～4。
* @param[in] col 字符列，范围0～20。
* @param[in] chr 可显示的ASCII字符，范围空格到波浪号（0x20～0x7e）。
* @return 无返回值；非法位置或字符不修改缓冲区。
 */
extern void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr);

/** @brief 按字符逐个绘制以 '\0' 结束的字符串；到列20后从下一行第0列继续。
* @param[in] row 起始行，范围0～4。
* @param[in] col 起始列，范围0～20。
* @param[in] chr 调用期间保持可读且不被修改的字符串，函数不改写它。
* @return 无返回值；空指针、非法位置或待显示部分含非法字符时缓冲区不变。
* @note 仅支持0x20～0x7e；到屏幕第4行末停止，超出部分不读取。
*       存储须可读至结束标记或剩余屏幕格数，以先达到者为止；不解析换行或UTF-8。
 */
extern void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr);

/** @brief 格式化文字并绘制在同一行的缓冲区中；不会自动刷新屏幕。
* @param[in] row 起始行，范围0～4，超范围直接返回。
* @param[in] col 起始列，范围0～20，超范围直接返回。
* @param[in] fmt 非空可读printf格式字符串，后续参数按格式提供，所引用对象保持可读至调用结束。
* @return 无返回值；非法位置、空格式、格式化失败或待显示部分含非法字符时缓冲区不变。
* @note 最多显示21-col个ASCII字符，超出当前行的内容丢弃；局部缓冲最多22字节含结束标记。
*       不清理本次未覆盖的旧字符格。格式属性不替代参数类型及可读存储要求，动态格式化耗时不保证。
 */
extern void OLED_printf(uint8_t row, uint8_t col, const char *fmt, ...) MC02_PRINTF_FORMAT(3, 4);


/** @brief 按页发送整个128×64显存缓冲区到 OLED 面板。
 * @return 无返回值；本接口不报告I2C传输结果。
 */
extern void OLED_refresh_gram(void);


/** @brief 清空缓冲区、绘制 RM 标志并刷新整屏。
 * @return 无返回值；本接口不报告I2C传输结果。
 */
extern void OLED_LOGO(void);
#endif
