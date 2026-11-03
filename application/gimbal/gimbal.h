/** @file gimbal.h
* @brief 云台应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"
#include "../robot_control_tasks.h"

typedef struct
{
    DJIMotorSummary_t yaw;
    DJIMotorSummary_t pitch;
} Gimbal_Motor_Summary_t;

/** @brief 复制yaw、pitch的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Gimbal_GetMotorSummary(Gimbal_Motor_Summary_t *summary);

#endif // GIMBAL_H
