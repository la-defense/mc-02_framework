/** @file omni.c
 * @brief 校验全向轮滚动方向，再交给共同投影计算。
 */
#include "omni.h"
#include <math.h>
#include <stddef.h>
// 全向轮正算和估速使用同一份几何校验。
static uint8_t OmniDirectionsValid(const OmniGeometry_t *g)
{
    if (g == NULL) return 0u;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const float length_squared = g->wheels[i].drive_x*g->wheels[i].drive_x +
                                     g->wheels[i].drive_y*g->wheels[i].drive_y;
        if (!isfinite(length_squared) || fabsf(length_squared-1.0f) > 0.0001f)
            return 0u;
    }
    return 1u;
}

ChassisKinematicsStatus_t OmniCalculate(const ChassisTwist_t *input,
    const OmniGeometry_t *g, ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    *output = (ChassisWheelSpeeds_t){0};
    if (!OmniDirectionsValid(g)) return CHASSIS_KINEMATICS_ARGUMENT;
    return ChassisKinematicsCalculate(input, g->wheels, output);
}

ChassisKinematicsStatus_t OmniPrepareProjector(const OmniGeometry_t *geometry, ChassisWheelProjector_t *projector)
{
    if (projector == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    if (!OmniDirectionsValid(geometry))
    {
        projector->ready = 0u; // 方向检查失败时也不能沿用旧系数。
        return CHASSIS_KINEMATICS_ARGUMENT;
    }
    return ChassisKinematicsPrepareProjector(geometry->wheels, projector);
}

ChassisKinematicsStatus_t OmniPrepareEstimator(const OmniGeometry_t *geometry, ChassisVelocityEstimator_t *estimator)
{
    if (estimator == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    if (!OmniDirectionsValid(geometry))
    {
        estimator->ready = 0u; // 方向检查失败时也放弃已有的系数。
        return CHASSIS_KINEMATICS_ARGUMENT;
    }
    return ChassisKinematicsPrepareEstimator(geometry->wheels, estimator);
}

ChassisKinematicsStatus_t OmniEstimate(const ChassisWheelSpeeds_t *input,
    const OmniGeometry_t *geometry, ChassisTwist_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    if (!OmniDirectionsValid(geometry))
    {
        *output = (ChassisTwist_t){0}; // 方向不合法时不把旧结果留给调用者。
        return CHASSIS_KINEMATICS_ARGUMENT;
    }
    return ChassisKinematicsEstimate(input, geometry->wheels, output);
}
