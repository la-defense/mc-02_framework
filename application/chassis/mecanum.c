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

ChassisKinematicsStatus_t MecanumPrepareProjector(const MecanumGeometry_t *g, ChassisWheelProjector_t *projector)
{
    if (projector == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    projector->ready = 0u; // 尺寸检查失败时不能使用已有投影。
    if (g == NULL || !isfinite(g->half_base_m) || g->half_base_m <= 0.0f ||
        !isfinite(g->half_track_m) || g->half_track_m <= 0.0f ||
        !isfinite(g->center_x_m) || !isfinite(g->center_y_m))
        return CHASSIS_KINEMATICS_ARGUMENT;
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT] = {
        {g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, -1, g->radius_m}, // 左前轮。
        {g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, 1, g->radius_m}, // 右前轮。
        {-g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, 1, g->radius_m}, // 左后轮。
        {-g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, -1, g->radius_m}}; // 右后轮。
    return ChassisKinematicsPrepareProjector(wheels, projector);
}
