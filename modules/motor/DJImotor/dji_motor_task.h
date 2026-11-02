/** @file dji_motor_task.h
 * @brief DJI 电机周期计算的轻量任务入口。
 */
#ifndef MC02_DJI_MOTOR_TASK_H
#define MC02_DJI_MOTOR_TASK_H
/** @brief 计算已注册电机的输出，并按总线提交周期帧。
 * @note 由唯一电机控制任务周期调用；注册与使能遵循电机驱动接口。
 */
void DJIMotorControl(void);
#endif
