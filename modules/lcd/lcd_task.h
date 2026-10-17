/** @file lcd_task.h
* @brief LCD 常驻任务入口，由 OSTaskInit() 创建，负责按键、显示和运行统计。
*/
#ifndef LCD_TASK_H
#define LCD_TASK_H

/** @brief 初始化显示后循环运行；每 50 ms 采按键，约每 200 ms 刷新数值。
* @param[in] argument 未使用的任务参数，可以为 NULL；仅作为 RTOS 任务入口调用。
*/
void StartLCDTASK(void const *argument);

#endif // LCD_TASK_H
