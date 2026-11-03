/** @file chassis.h
* @brief 底盘应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "dji_motor.h"
#include "../robot_control_tasks.h"

typedef struct
{
    DJIMotorSummary_t lf;
    DJIMotorSummary_t rf;
    DJIMotorSummary_t lb;
    DJIMotorSummary_t rb;
} Chassis_Motor_Summary_t;

/** @brief 复制四个轮子的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary);

#endif // CHASSIS_H
