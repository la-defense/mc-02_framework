/** @file half_steering.h
 * @brief 半舵占位；尚无可用控制实现，生产构建选择此类型时拒绝。
 */
#ifndef MC02_HALF_STEERING_H
#define MC02_HALF_STEERING_H
#include "chassis_kinematics.h"
/** @brief 报告此类型尚未实现；所有输入保持不变。
 * @param[in] input 预留的底盘中心速度，允许NULL。
 * @param[out] output 非NULL时清零。
 * @return 始终返回UNSUPPORTED，不能据此使能执行器。
 */
ChassisKinematicsStatus_t HalfSteeringCalculate(const ChassisTwist_t *input, ChassisWheelSpeeds_t *output);
#endif
