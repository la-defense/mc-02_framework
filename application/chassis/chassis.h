/** @file chassis.h
* @brief 底盘应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "dji_motor.h"
#include "chassis_kinematics.h"
#include "../robot_control_tasks.h"

typedef struct
{
    DJIMotorSummary_t lf;
    DJIMotorSummary_t rf;
    DJIMotorSummary_t lb;
    DJIMotorSummary_t rb;
} Chassis_Motor_Summary_t;

/** @brief 由四轮反馈估算的底盘速度，使用前、左、上坐标。 */
typedef struct {
    ChassisTwist_t twist; /**< 底盘中心平移速度 m/s，绕竖轴转速 rad/s。 */
    uint32_t source_time_ms; /**< 参与计算的最早轮速接收时间，0 是合法时间。 */
} ChassisVelocityFeedback_t;

/** @brief 读取本板最近一次有效的全向底盘速度估计。
 * @param[out] output 底盘中心速度及最早来源时间；失败时非空输出清零。
 * @return 1 表示四轮估计仍在原有效期内，0 表示缺少估计、配置不支持或已过期。
 * @note 任务上下文使用；四轮估计以滚动模型为依据，无法单独识别打滑。
 */
uint8_t Chassis_GetVelocityFeedback(ChassisVelocityFeedback_t *output);

/** @brief 复制四个轮子的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary);

#endif // CHASSIS_H
