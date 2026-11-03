/** @file ht_motor_task.h
 * @brief HT 电机独立控制任务的启动入口。
 */
#ifndef MC02_HT_MOTOR_TASK_H
#define MC02_HT_MOTOR_TASK_H
#include <stdint.h>

/** @brief 为每个已注册 HT 电机创建约每 1 ms 运行的任务。
 * @return 全部创建成功或无电机时返回 1；创建失败返回 0。
 * @note 调度器启动前调用；失败时已创建的任务保留，再次调用只创建缺失的任务。
 */
uint8_t HTMotorControlInit(void);
#endif
