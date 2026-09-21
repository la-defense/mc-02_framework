#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"

typedef struct
{
    DJIMotorSummary_t friction_l;
    DJIMotorSummary_t friction_r;
    DJIMotorSummary_t loader;
} Shoot_Motor_Summary_t;

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief 获取摩擦轮/拨弹盘电机摘要(供LCD显示)
 */
void Shoot_GetMotorSummary(Shoot_Motor_Summary_t *summary);

#endif // SHOOT_H
