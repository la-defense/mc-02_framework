/**
 * @file lcd_ui.h
 * @brief 四页 LCD 界面的初始化、绘制、刷新、恢复和按键提示入口。
 * @note 仅由 LCD 显示任务按顺序调用；页面、冻结状态仍由现有任务逻辑管理。
 */
#ifndef MC02_LCD_UI_H
#define MC02_LCD_UI_H

#include <stdint.h>
#include "lcd.h"

#ifndef MC02_BENCH_LCD_DIAGNOSTICS
#define MC02_BENCH_LCD_DIAGNOSTICS 0
#endif

#define LCD_PAGE_COUNT 4u

/**
 * @brief 初始化面板、清屏并绘制第 0 页，完成后打开背光。
 * @return LCD_OK 或具体初始化、期限及 SPI 错误；失败后界面不可用且背光关闭。
 * @note 仅由 LCD 任务在 GPIO、SPI1 和 HAL 毫秒时基初始化后调用。
 */
LCDStatus_t LCD_UI_Init(void);
/**
 * @brief 对 SPI 做一次停止确认，再重初始化、清屏并重画当前页。
 * @return LCD_OK 或停止、初始化、绘制错误；停止失败后总线隔离，等待系统重新初始化。
 * @note 调用后只有整页发送成功才会重新打开背光。
 */
LCDStatus_t LCD_UI_Recover(void);
/**
 * @brief 重画指定页标题和标签，并使值区缓存失效。
 * @param[in] page 页号，任意 uint8_t 值按 LCD_PAGE_COUNT 取余。
 * @return LCD_OK 或具体范围、期限及发送错误；部分写入时界面失效并关闭背光。
 * @note 最外层调用期限为 250ms，嵌套绘图共用此期限。
 */
LCDStatus_t LCD_UI_DrawStatic(uint8_t page);
/**
 * @brief 按变化刷新指定页的值，发送成功后才确认对应行缓存。
 * @param[in] page 页号，任意 uint8_t 值按 LCD_PAGE_COUNT 取余。
 * @return LCD_OK 或具体刷新错误；失败不确认正在绘制的行。
 * @note 最外层调用期限为 20ms，嵌套绘图共用此期限。
 */
LCDStatus_t LCD_UI_UpdateValues(uint8_t page);
/**
 * @brief 更新冻结标记并重画标题栏。
 * @param[in] frozen 非 0 表示冻结，0 表示继续刷新数值。
 * @return LCD_OK 或具体发送错误；失败时按统一规则关闭背光并使界面不可用。
 */
LCDStatus_t LCD_UI_SetFrozen(uint8_t frozen);
/**
 * @brief 清除值区缓存记录，使下次刷新重新绘制所有值。
 * @note 仅供 LCD 显示任务在整页被清理或覆盖后调用，不发送数据。
 */
void LCD_UI_InvalidateValueCache(void);
/**
 * @brief 绘制标定过程或结果提示。
 * @param[in] msg 0 清除提示并恢复当前页，1 显示进行中，2 显示成功，其余值显示失败。
 * @return LCD_OK 或具体绘图错误；失败后界面不可用且背光关闭。
 */
LCDStatus_t LCD_UI_ShowCalibMsg(uint8_t msg);

/**
 * @brief 绘制 bench_safe LCD 诊断画面之一。
 * @param[in] pattern 0～3，依次为边缘／颜色、ASCII 字号、中文字形和数值。
 * @return LCD_OK 或具体绘图状态；失败后界面不可用且背光关闭。
 * @note 仅在 MC02_BENCH_LCD_DIAGNOSTICS=1 的禁热诊断固件中声明；每次只绘一张。
 */
#if MC02_BENCH_LCD_DIAGNOSTICS
LCDStatus_t LCD_UI_DrawDiagnosticPattern(uint8_t pattern);
#endif

#endif /* MC02_LCD_UI_H */
