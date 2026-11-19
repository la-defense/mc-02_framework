/** @file chassis_kinematics.c
 * @brief 使用同一组四轮投影计算轮速和估计底盘速度。
 */
#include "chassis_kinematics.h"
#include <math.h>
#include <float.h>
#include <stddef.h>
// 从同一份轮组几何建立投影和对应的正规方程系数 AᵀA。
static uint8_t BuildProjection(const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT],
    double projection[CHASSIS_WHEEL_COUNT][3], double gram[3][3])
{
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const ChassisWheelGeometry_t *w = &wheels[i];
        if (!isfinite(w->x_m) || !isfinite(w->y_m) ||
            !isfinite(w->drive_x) || !isfinite(w->drive_y) ||
            !isfinite(w->radius_m) || w->radius_m <= 0.0f)
            return 0u; // 含无效尺寸或方向时整个轮组无法计算。
        double *axis = projection[i]; // 每轮对应底盘前进、左移和绕竖轴转动三项。
        axis[0] = w->drive_x; // 前进分量投影到滚动方向。
        axis[1] = w->drive_y; // 左移分量投影到滚动方向。
        axis[2] = (double)w->x_m * w->drive_y - (double)w->y_m * w->drive_x; // 转动在轮位产生切向速度。
        // 把这一轮的投影乘积累加到正规方程，供满秩判断和估计使用。
        for (unsigned row = 0u; row < 3u; ++row)
            for (unsigned col = 0u; col < 3u; ++col)
                gram[row][col] += axis[row] * axis[col];
    }
    return 1u;
}

// 计算对称矩阵的伴随矩阵并检查行列式，接近缺失自由度时返回 0。
static double ProjectionDeterminant(double gram[3][3], double adj[3][3])
{
    const double a = gram[0][0], b = gram[0][1], c = gram[0][2]; // 第一行保存前进与各轴的投影乘积。
    const double d = gram[1][1], e = gram[1][2], f = gram[2][2]; // 对称矩阵中尚未读取的三个分量。
    adj[0][0] = d * f - e * e; // 去掉第一个自由度对应的行和列。
    adj[0][1] = adj[1][0] = c * e - b * f; // 前进和横移对应的带符号余子式。
    adj[0][2] = adj[2][0] = b * e - c * d; // 前进和转动对应的带符号余子式。
    adj[1][1] = a * f - c * c; // 去掉横移对应的行和列。
    adj[1][2] = adj[2][1] = b * c - a * e; // 横移和转动对应的带符号余子式。
    adj[2][2] = a * d - b * b; // 去掉转动对应的行和列。
    const double determinant = a * adj[0][0] + b * adj[0][1] + c * adj[0][2]; // 沿第一行展开。
    const double scale = a * d * f; // 人为缩放各自由度时，用该乘积检查相对退化程度。
    return isfinite(scale) && isfinite(determinant) && scale > 0.0 &&
           determinant > 1e-6 * scale ? determinant : 0.0;
}

ChassisKinematicsStatus_t ChassisKinematicsCalculate(const ChassisTwist_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    *output = (ChassisWheelSpeeds_t){0};
    if (input == NULL || wheels == NULL || !isfinite(input->vx_m_s) ||
        !isfinite(input->vy_m_s) || !isfinite(input->wz_rad_s))
        return CHASSIS_KINEMATICS_ARGUMENT;
    double projection[CHASSIS_WHEEL_COUNT][3], gram[3][3] = {{0}}, adj[3][3];
    if (!BuildProjection(wheels, projection, gram) || ProjectionDeterminant(gram, adj) == 0.0)
        return CHASSIS_KINEMATICS_ARGUMENT;
    ChassisWheelSpeeds_t candidate = {0};
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const double speed = (projection[i][0] * input->vx_m_s + projection[i][1] * input->vy_m_s +
                              projection[i][2] * input->wz_rad_s) / wheels[i].radius_m;
        if (!isfinite(speed) || fabs(speed) > FLT_MAX) return CHASSIS_KINEMATICS_ARGUMENT;
        candidate.rad_s[i] = (float)speed;
    }
    *output = candidate;
    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_t ChassisKinematicsEstimate(const ChassisWheelSpeeds_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisTwist_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    ChassisVelocityEstimator_t estimator; // 单次调用也复用相同的几何和估速步骤。
    if (input != NULL && ChassisKinematicsPrepareEstimator(wheels, &estimator) == CHASSIS_KINEMATICS_OK)
        return ChassisKinematicsEstimatePrepared(input, &estimator, output);
    *output = (ChassisTwist_t){0}; // 参数无效时清掉整组结果。
    return CHASSIS_KINEMATICS_ARGUMENT;
}

ChassisKinematicsStatus_t ChassisKinematicsPrepareEstimator(
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisVelocityEstimator_t *estimator)
{
    if (estimator == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    estimator->ready = 0u; // 重配失败时也不能继续沿用旧几何的系数。
    if (wheels == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    double projection[CHASSIS_WHEEL_COUNT][3], gram[3][3] = {{0}}, adj[3][3];
    if (!BuildProjection(wheels, projection, gram)) return CHASSIS_KINEMATICS_ARGUMENT;
    const double determinant = ProjectionDeterminant(gram, adj); // 满秩才可求三个速度分量。
    if (determinant == 0.0) return CHASSIS_KINEMATICS_ARGUMENT;

    for (unsigned axis = 0u; axis < 3u; ++axis) // 按输出方向生成系数矩阵的每一行。
        for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) // 每列表示一个轮速对该方向的贡献。
        {
            const double projected = adj[axis][0] * projection[i][0] + adj[axis][1] * projection[i][1] +
                                     adj[axis][2] * projection[i][2]; // 对应 (AᵀA) 伴随矩阵乘 Aᵀ。
            const double gain = (projected / determinant) * wheels[i].radius_m; // 求逆并计入轮轴角速度到轮缘速度的换算。
            if (!isfinite(gain)) return CHASSIS_KINEMATICS_ARGUMENT; // 所有系数须可用，部分结果保持不可读。
            estimator->gain[axis][i] = gain; // 按固定轮序保存，周期计算无须再读取几何。
        }
    estimator->ready = 1u; // 全部结果有效后才开放读取。
    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_t ChassisKinematicsEstimatePrepared(const ChassisWheelSpeeds_t *input,
    const ChassisVelocityEstimator_t *estimator, ChassisTwist_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    *output = (ChassisTwist_t){0}; // 任一输入或分量错误时清掉整组结果。
    if (input == NULL || estimator == NULL || estimator->ready != 1u) return CHASSIS_KINEMATICS_ARGUMENT;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
        if (!isfinite(input->rad_s[i])) return CHASSIS_KINEMATICS_ARGUMENT; // 一轮无效就不执行矩阵乘法。
    float estimate[3]; // 在三个分量都通过有限数检查后一次交付。
    for (unsigned axis = 0u; axis < 3u; ++axis)
    {
        double speed = 0.0; // 从零累加四个轮速的贡献。
        for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
            speed += estimator->gain[axis][i] * input->rad_s[i]; // 每个轴只需计算固定系数与轮速的点积。
        if (!isfinite(speed) || fabs(speed) > FLT_MAX) return CHASSIS_KINEMATICS_ARGUMENT;
        estimate[axis] = (float)speed;
    }
    *output = (ChassisTwist_t){estimate[0], estimate[1], estimate[2]};
    return CHASSIS_KINEMATICS_OK;
}

ChassisKinematicsStatus_t ChassisKinematicsIntegrate(ChassisLocalPose_t *pose,
    const ChassisTwist_t *previous, const ChassisTwist_t *current, float dt_s)
{
    if (pose == NULL || previous == NULL || current == NULL || !isfinite(dt_s) || dt_s <= 0.0f)
        return CHASSIS_KINEMATICS_ARGUMENT; // 缺少输入或时间没有前进时不修改位姿。
    const float dx = (0.5f * previous->vx_m_s + 0.5f * current->vx_m_s) * dt_s; // 以两端平均前进速度求无转动时的位移。
    const float dy = (0.5f * previous->vy_m_s + 0.5f * current->vy_m_s) * dt_s; // 横移也以底盘自身的左侧为正。
    const float half_turn = (0.5f * previous->wz_rad_s + 0.5f * current->wz_rad_s) * dt_s * 0.5f; // 取本段朝向变化的一半。
    if (!isfinite(dx) || !isfinite(dy) || !isfinite(half_turn) || !isfinite(pose->yaw_rad))
        return CHASSIS_KINEMATICS_ARGUMENT; // 三角函数只接收有限角度，异常时保留上次结果。
    const float scale = fabsf(half_turn) < 0.0001f ? 1.0f : sinf(half_turn) / half_turn; // 弧长换为弦长；接近直行时用极限值避免除零。
    const float middle_yaw = pose->yaw_rad + half_turn; // 弦的方向相对区间中点朝向计算。
    const float c = cosf(middle_yaw), s = sinf(middle_yaw); // 将底盘自身的位移转到本段起点的坐标。
    const ChassisLocalPose_t next = {
        .x_m = pose->x_m + scale * (c * dx - s * dy), // 累加到起点坐标中的前向位移。
        .y_m = pose->y_m + scale * (s * dx + c * dy), // 累加到起点坐标中的左向位移。
        .yaw_rad = remainderf(pose->yaw_rad + 2.0f * half_turn, 6.28318530718f), // 保持在约 ±π 内，避免角度随圈数增长。
    };
    if (!isfinite(next.x_m) || !isfinite(next.y_m) || !isfinite(next.yaw_rad))
        return CHASSIS_KINEMATICS_ARGUMENT; // 任一分量溢出时放弃本段积分。
    *pose = next; // 只在三个分量都通过检查后一起更新。
    return CHASSIS_KINEMATICS_OK;
}
