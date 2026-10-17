/** @file chassis.h
* @brief 底盘应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
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

/** @brief 注册底盘所需电机和消息，初始化完成后才运行任务。
* @return 1 为所需资源均已取得，0 为至少一项失败；由 RobotInit() 停止后续启动并处理失败。
* @note 先准备所需 BSP，再在 RTOS 启动前由 RobotInit() 调用一次。
 */
uint8_t ChassisInit(void);

/** @brief 处理一轮底盘控制；先读取不超过 50 ms 的命令，无有效命令则停止相应输出。
* @note 初始化成功后由 RobotTask() 周期调用，本函数本身不包含常驻循环。
 */
void ChassisTask();

/** @brief 复制四个轮子的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary);

#endif // CHASSIS_H
