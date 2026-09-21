#ifndef CHASSIS_H
#define CHASSIS_H

#include "dji_motor.h"

typedef struct
{
    DJIMotorSummary_t lf;
    DJIMotorSummary_t rf;
    DJIMotorSummary_t lb;
    DJIMotorSummary_t rb;
} Chassis_Motor_Summary_t;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/**
 * @brief 获取四个底盘电机的只读摘要(供LCD显示)
 */
void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary);

#endif // CHASSIS_H
