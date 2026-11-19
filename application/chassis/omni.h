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
/** @brief 检查全向轮方向并准备周期目标轮速的投影。
 * @param[in] geometry 实物位置、单位滚动方向和轮径。
 * @param[out] projector 保存到几何改变或底盘停止；失败时标为不可用。
 * @return OK 或 ARGUMENT。
 */
ChassisKinematicsStatus_t OmniPrepareProjector(const OmniGeometry_t *geometry, ChassisWheelProjector_t *projector);
/** @brief 按全向轮几何准备周期估速需要的固定系数。
 * @param[in] geometry 实物位置、单位滚动方向和轮径。
 * @param[out] estimator 由调用者保存到几何改变或底盘停止；失败时标为不可用。
 * @return OK 或 ARGUMENT；无效方向、轮径或退化几何均拒绝。
 */
ChassisKinematicsStatus_t OmniPrepareEstimator(const OmniGeometry_t *geometry, ChassisVelocityEstimator_t *estimator);
/** @brief 用全向轮输出轴速度估计底盘中心速度。
 * @param[in] input 统一轮序的实测轮轴 rad/s。
 * @param[in] geometry 实物位置、单位滚动方向和轮径。
 * @param[out] output 底盘 m/s、rad/s；失败时非空输出清零。
 * @return OK 或 ARGUMENT；近退化几何和非有限轮速均拒绝。
 */
ChassisKinematicsStatus_t OmniEstimate(const ChassisWheelSpeeds_t *input,
    const OmniGeometry_t *geometry, ChassisTwist_t *output);
#endif
