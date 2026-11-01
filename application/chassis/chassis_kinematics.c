/** @file chassis_kinematics.c
 * @brief 检查轮组是否能产生三个独立方向，再一次交付全部轮速。
 */
#include "chassis_kinematics.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
ChassisKinematicsStatus_t ChassisKinematicsCalculate(const ChassisTwist_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    memset(output, 0, sizeof(*output));
    if (input == NULL || wheels == NULL || !isfinite(input->vx_m_s) ||
        !isfinite(input->vy_m_s) || !isfinite(input->wz_rad_s))
        return CHASSIS_KINEMATICS_ARGUMENT;
    double gram[3][3] = {{0}};
    ChassisWheelSpeeds_t candidate = {0};
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const ChassisWheelGeometry_t *w = &wheels[i];
        if (!isfinite(w->x_m) || !isfinite(w->y_m) ||
            !isfinite(w->drive_x) || !isfinite(w->drive_y) ||
            !isfinite(w->radius_m) || w->radius_m <= 0.0f)
            return CHASSIS_KINEMATICS_ARGUMENT;
        const double a[3] = {w->drive_x, w->drive_y,
            (double)w->x_m * w->drive_y - (double)w->y_m * w->drive_x};
        for (unsigned row = 0u; row < 3u; ++row)
            for (unsigned col = 0u; col < 3u; ++col)
                gram[row][col] += a[row] * a[col];
        candidate.rad_s[i] = (float)((a[0] * input->vx_m_s + a[1] * input->vy_m_s +
                                      a[2] * input->wz_rad_s) / w->radius_m);
        if (!isfinite(candidate.rad_s[i])) return CHASSIS_KINEMATICS_ARGUMENT;
    }
    const double determinant = gram[0][0]*(gram[1][1]*gram[2][2]-gram[1][2]*gram[2][1]) -
        gram[0][1]*(gram[1][0]*gram[2][2]-gram[1][2]*gram[2][0]) +
        gram[0][2]*(gram[1][0]*gram[2][1]-gram[1][1]*gram[2][0]);
    const double scale = gram[0][0] * gram[1][1] * gram[2][2];
    // 平行轮组或缺少转动能力时，不把投影结果当作完整底盘计算成功。
    if (!isfinite(scale) || !isfinite(determinant) || scale <= 0.0 ||
        determinant <= 1e-6 * scale) return CHASSIS_KINEMATICS_ARGUMENT;
    *output = candidate;
    return CHASSIS_KINEMATICS_OK;
}
