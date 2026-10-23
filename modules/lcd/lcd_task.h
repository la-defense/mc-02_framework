/** @file lcd_task.h
* @brief LCD 常驻任务入口，由 OSTaskInit() 创建，负责按键、显示和运行统计。
*/
#ifndef LCD_TASK_H
#define LCD_TASK_H

#include <stdint.h>
#include "lcd.h"

typedef enum
{
    LCD_TASK_DIAG_DISABLED = 0,
    LCD_TASK_DIAG_IDLE,
    LCD_TASK_DIAG_RUNNING,
    LCD_TASK_DIAG_COMPLETE,
    LCD_TASK_DIAG_FAILED
} LCDTaskDiagnosticState_t;

/** @brief 初始化显示后循环运行；每 50 ms 采按键，约每 200 ms 刷新数值。
* @param[in] argument 未使用的任务参数，可以为 NULL；仅作为 RTOS 任务入口调用。
*/
void StartLCDTASK(void const *argument);

/**
 * @brief 复制本次可选 LCD 诊断的运行状态，不推进诊断步骤。
 * @param[out] output 非空枚举输出；失败保持原值。
 * @return LCD_OK 或 LCD_BAD_ARGUMENT。
 */
LCDStatus_t LCDTaskGetDiagnosticState(LCDTaskDiagnosticState_t *output);

#endif // LCD_TASK_H
