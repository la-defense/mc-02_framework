/** @file steering.c
 * @brief 舵轮占位实现，始终报告不支持并清零输出。
 */
#include "steering.h"
#include <stddef.h>
#include <string.h>
ChassisKinematicsStatus_t SteeringCalculate(const ChassisTwist_t *input, ChassisWheelSpeeds_t *output)
{
    (void)input;
    if (output != NULL) memset(output, 0, sizeof(*output));
    return CHASSIS_KINEMATICS_UNSUPPORTED;
}
