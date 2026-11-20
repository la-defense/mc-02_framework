/** @file mecanum.c
 * @brief 构造麦轮投影，轮序为左前、右前、左后、右后。
 */
#include "mecanum.h"
#include <math.h>
#include <stddef.h>
ChassisKinematicsStatus_t MecanumCalculate(const ChassisTwist_t *input,
    const MecanumGeometry_t *g, ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    ChassisWheelProjector_t projector;
    if (input != NULL && MecanumPrepareProjector(g, &projector) == CHASSIS_KINEMATICS_OK)
        return ChassisKinematicsCalculatePrepared(input, &projector, output);
    *output = (ChassisWheelSpeeds_t){0}; // 缺少几何或输入时清掉所有旧轮速。
    return CHASSIS_KINEMATICS_ARGUMENT;
}

// 目标轮速和速度反馈使用同一套轮位与滚动方向。
static uint8_t MecanumBuildWheels(const MecanumGeometry_t *g, ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT])
{
    if (g == NULL || !isfinite(g->half_base_m) || g->half_base_m <= 0.0f ||
        !isfinite(g->half_track_m) || g->half_track_m <= 0.0f ||
        !isfinite(g->center_x_m) || !isfinite(g->center_y_m))
        return 0u; // 长度和偏移必须能确定四个轮位。
    wheels[0] = (ChassisWheelGeometry_t){g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, -1, g->radius_m}; // 左前轮，滚动投影为前、右。
    wheels[1] = (ChassisWheelGeometry_t){g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, 1, g->radius_m}; // 右前轮，滚动投影为前、左。
    wheels[2] = (ChassisWheelGeometry_t){-g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, 1, g->radius_m}; // 左后轮，滚动投影为前、左。
    wheels[3] = (ChassisWheelGeometry_t){-g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, -1, g->radius_m}; // 右后轮，滚动投影为前、右。
    return 1u; // 轮半径由公共运动学检查。
}

ChassisKinematicsStatus_t MecanumPrepareProjector(const MecanumGeometry_t *g, ChassisWheelProjector_t *projector)
{
    if (projector == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    projector->ready = 0u; // 尺寸检查失败时不能使用已有投影。
    ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT];
    if (!MecanumBuildWheels(g, wheels)) return CHASSIS_KINEMATICS_ARGUMENT;
    return ChassisKinematicsPrepareProjector(wheels, projector);
}

ChassisKinematicsStatus_t MecanumPrepareEstimator(const MecanumGeometry_t *g, ChassisVelocityEstimator_t *estimator)
{
    if (estimator == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    estimator->ready = 0u; // 重新准备失败时不沿用旧车型系数。
    ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT];
    if (!MecanumBuildWheels(g, wheels)) return CHASSIS_KINEMATICS_ARGUMENT;
    return ChassisKinematicsPrepareEstimator(wheels, estimator);
}
