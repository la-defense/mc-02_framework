/** @file mecanum.c
 * @brief 构造麦轮投影，轮序为左前、右前、左后、右后。
 */
#include "mecanum.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
ChassisKinematicsStatus_t MecanumCalculate(const ChassisTwist_t *input,
    const MecanumGeometry_t *g, ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    memset(output, 0, sizeof(*output));
    if (g == NULL || !isfinite(g->half_base_m) || g->half_base_m <= 0.0f ||
        !isfinite(g->half_track_m) || g->half_track_m <= 0.0f ||
        !isfinite(g->center_x_m) || !isfinite(g->center_y_m))
        return CHASSIS_KINEMATICS_ARGUMENT;
    const ChassisWheelGeometry_t wheels[4] = {
        {g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, -1, g->radius_m},
        {g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, 1, g->radius_m},
        {-g->half_base_m-g->center_x_m, g->half_track_m-g->center_y_m, 1, 1, g->radius_m},
        {-g->half_base_m-g->center_x_m, -g->half_track_m-g->center_y_m, 1, -1, g->radius_m}};
    return ChassisKinematicsCalculate(input, wheels, output);
}
