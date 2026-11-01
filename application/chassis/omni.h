/** @file omni.h
 * @brief 四轮全向轮计算；滚动方向由实物安装配置给出，不预设安装角。
 */
#ifndef MC02_OMNI_H
#define MC02_OMNI_H
#include "chassis_kinematics.h"
typedef struct {
    ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT];
} OmniGeometry_t;
/** @brief 按四轮位置和单位滚动方向计算轮轴速度。
 * @param[in] input 底盘中心速度，m/s、rad/s，不可为NULL。
 * @param[in] geometry 四轮配置；drive_x平方加drive_y平方须在1的0.0001范围内。
 * @param[out] output 轮轴rad/s；失败时非NULL输出清零。
 * @return OK或ARGUMENT；轮组须能独立控制前后、左右及转动。
 */
ChassisKinematicsStatus_t OmniCalculate(const ChassisTwist_t *input,
    const OmniGeometry_t *geometry, ChassisWheelSpeeds_t *output);
#endif
