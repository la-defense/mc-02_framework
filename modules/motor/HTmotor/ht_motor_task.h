/** @file ht_motor_task.h
 * @brief HT 电机独立控制任务的启动入口。
 */
#ifndef MC02_HT_MOTOR_TASK_H
#define MC02_HT_MOTOR_TASK_H

/** @brief 为每个已注册 HT 电机创建约每 1 ms 运行的任务，无电机时返回。
 * @note 注册完成后只调用一次；当前实现不检查任务创建失败，重复调用会新增任务。
 */
void HTMotorControlInit(void);
#endif
