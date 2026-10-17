/** @file gimbal.h
* @brief 云台应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_motor.h"

typedef struct
{
    DJIMotorSummary_t yaw;
    DJIMotorSummary_t pitch;
} Gimbal_Motor_Summary_t;

/** @brief 注册云台所需电机和消息，初始化完成后才运行任务。
* @return 1 为所需资源均已取得，0 为至少一项失败；由 RobotInit() 停止后续启动并处理失败。
* @note 先准备所需 BSP，再在 RTOS 启动前由 RobotInit() 调用一次；INS 算法随后由专属任务启动。
 */
uint8_t GimbalInit(void);

/** @brief 处理一轮云台控制；先读取不超过 50 ms 的命令，无有效命令则停止相应输出。
* @note 初始化成功后由 RobotTask() 周期调用，本函数本身不包含常驻循环。
 */
void GimbalTask();

/** @brief 复制yaw、pitch的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Gimbal_GetMotorSummary(Gimbal_Motor_Summary_t *summary);

#endif // GIMBAL_H
