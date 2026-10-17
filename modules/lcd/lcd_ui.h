/** @file lcd_ui.h
 * @brief 四页 LCD 界面：标题和标签在换页时画，数值在周期更新时按变化重画。
 * @note 由 LCD 任务顺序调用；先初始化，换页时先 DrawStatic() 再 UpdateValues()。
 */
#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>

#define LCD_PAGE_COUNT 4u

/** @brief 初始化屏幕、清屏并显示第 0 页，同时清除冻结标记。 */
void LCD_UI_Init(void);
/** @brief 必要时恢复 SPI，再初始化面板，清屏并重画当前页；保留冻结标记。 */
void LCD_UI_Recover(void);
/** @brief 重画标题及标签，使所有值区在下次更新时重画。
 * @param[in] page 页号，内部对 LCD_PAGE_COUNT 取余；0 系统、1 IMU、2 功率、3 电机。
 */
void LCD_UI_DrawStatic(uint8_t page);
/** @brief 读取并更新所选页数值，内容未变的行跳过；本函数本身不检查冻结标记。
 * @param[in] page 页号，内部对 LCD_PAGE_COUNT 取余，应与已绘制标签所在页一致。
 */
void LCD_UI_UpdateValues(uint8_t page);
/** @brief 改写冻结标记并重画标题；是否暂停值区更新由调用方决定。
 * @param[in] frozen 非 0 显示冻结标记，为 0 清除；不直接停止其他任务。
 */
void LCD_UI_SetFrozen(uint8_t frozen);
/** @brief 清除各行已绘制的记录，使下一次数值更新重新绘制；画面被覆盖后须调用。 */
void LCD_UI_InvalidateValueCache(void);
/** @brief 在当前页中间显示标定提示，为 0 时重画当前页。
 * @param[in] msg 0 清除、1 标定中、2 成功，其他值显示失败；不执行标定本身。
 */
void LCD_UI_ShowCalibMsg(uint8_t msg);

#endif // LCD_UI_H
