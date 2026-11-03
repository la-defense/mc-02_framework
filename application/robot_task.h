/** @file robot_task.h
 * @brief 声明由任务创建模块使用的现有常驻任务入口。
 */
#ifndef MC02_ROBOT_TASK_H
#define MC02_ROBOT_TASK_H
#include "config/build_selection.h"

/** @brief 初始化 INS 后约每 1 ms 处理采样和姿态；失败时只维持诊断循环。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartINSTASK(void const *argument);
/** @brief 约每 1 ms 派发通信；仅机器人配置会继续运行电机计算和发送。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartMOTORTASK(void const *argument);
/** @brief 约每 10 ms 检查设备和任务，全部正常时才喂狗。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartDAEMONTASK(void const *argument);
/** @brief 约每 5 ms 运行整车控制和诊断。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartROBOTTASK(void const *argument);
#if MC02_FEATURE_UI
/** @brief 等待新鲜裁判机器人编号并周期运行 UI；每次循环至少等待 1 ms。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartUITASK(void const *argument);
#endif

#endif
