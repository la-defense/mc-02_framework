/** @file lk_motor_task.h
 * @brief LK 电机周期计算的轻量任务入口。
 */
#ifndef MC02_LK_MOTOR_TASK_H
#define MC02_LK_MOTOR_TASK_H
/** @brief 计算各电机并为每条非空总线提交八字节 0x280 帧。
 * @note 由唯一电机控制任务周期调用。编号 1～4 对应各自两字节槽位；空槽和停止电机写零。
 * 输入、反馈、PID、前馈或计算异常时清该电机的槽位和控制历史；发送失败后下一周期重新计算。
 * 本机提交不代表对端已收到。
 */
void LKMotorControl(void);
#endif
