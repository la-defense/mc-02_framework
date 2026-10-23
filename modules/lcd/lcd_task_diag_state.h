/** @file lcd_task_diag_state.h
 * @brief LCD 诊断状态与常规按键、页面刷新的占用关系。
 * 仅供 LCD 任务和对应主机测试使用。
 */
#ifndef LCD_TASK_DIAG_STATE_H
#define LCD_TASK_DIAG_STATE_H

#include "lcd_task.h"

/** @brief 判断四页自测当前是否占用 LCD 界面。
 * @param[in] state 当前诊断状态。
 * @return 仅在四页自测运行中返回 1；完成或失败后返回 0，让任务恢复按键和页面刷新。
 */
static inline uint8_t LCDTaskDiagnosticOwnsUi(LCDTaskDiagnosticState_t state)
{
    return (uint8_t)(state == LCD_TASK_DIAG_RUNNING);
}

#endif /* LCD_TASK_DIAG_STATE_H */
