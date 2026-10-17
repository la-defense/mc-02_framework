/**
 * @file motor_task.h
 * @author neozng
 * @brief  对所有电机,舵机等控制任务的进一步封装,MotorControlTask()将在操作系统中按一定频率调用
 * @version beta
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H


/** @brief 先派发 CAN、UART、USB 接收，再在正式配置计算 DJI、LK 输出；裸板配置跳过电机计算。
* @note 由 MOTOR 任务约每 1 ms 调用，全工程仅此处周期派发接收；实际延迟仍取决于调度。
 */
void MotorControlTask();

#endif // !MOTOR_TASK_H

