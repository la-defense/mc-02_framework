/** @file omni.c
 * @brief 校验全向轮滚动方向，再交给共同投影计算。
 */
#include "omni.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
ChassisKinematicsStatus_t OmniCalculate(const ChassisTwist_t *input,
    const OmniGeometry_t *g, ChassisWheelSpeeds_t *output)
{
    if (output == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    memset(output, 0, sizeof(*output));
    if (g == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const float length_squared = g->wheels[i].drive_x*g->wheels[i].drive_x +
                                     g->wheels[i].drive_y*g->wheels[i].drive_y;
        if (!isfinite(length_squared) || fabsf(length_squared-1.0f) > 0.0001f)
            return CHASSIS_KINEMATICS_ARGUMENT;
    }
    return ChassisKinematicsCalculate(input, g->wheels, output);
}
