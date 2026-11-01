/** @file wheel_leg.c
 * @brief 轮腿占位实现，始终报告不支持并清零输出。
 */
#include "wheel_leg.h"
#include <stddef.h>
#include <string.h>
ChassisKinematicsStatus_t WheelLegCalculate(const ChassisTwist_t *input, ChassisWheelSpeeds_t *output)
{
    (void)input;
    if (output != NULL) memset(output, 0, sizeof(*output));
    return CHASSIS_KINEMATICS_UNSUPPORTED;
}
