#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"

typedef struct
{
    DJIMotorSummary_t yaw;
    DJIMotorSummary_t pitch;
} Gimbal_Motor_Summary_t;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

/**
 * @brief 获取云台yaw/pitch电机摘要(供LCD显示)
 */
void Gimbal_GetMotorSummary(Gimbal_Motor_Summary_t *summary);

#endif // GIMBAL_H
