/** @file shoot.h
* @brief 发射机构应用入口：初始化资源，周期接收控制消息，并提供 LCD 所需的电机信息。
 */
#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"
#include "../robot_control_tasks.h"

typedef struct
{
    DJIMotorSummary_t friction_l; /**< 第一套摩擦轮组的第1个电机。 */
    DJIMotorSummary_t friction_r; /**< 第一套摩擦轮组的第2个电机。 */
    DJIMotorSummary_t friction_extra[2]; /**< 第3、4个摩擦轮电机。 */
    uint8_t friction_count; /**< 当前配置启用的摩擦轮数量，范围2～4。 */
    DJIMotorSummary_t loader; /**< 拨弹盘电机。 */
} Shoot_Motor_Summary_t;

/** @brief 复制摩擦轮和拨弹盘的当前电机信息，供 LCD 使用。
* @param[out] summary 输出对象，NULL 时不处理；先清零，未注册电机项保持为 0。
 */
void Shoot_GetMotorSummary(Shoot_Motor_Summary_t *summary);

#endif // SHOOT_H
