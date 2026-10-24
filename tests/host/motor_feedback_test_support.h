/**
 * @file motor_feedback_test_support.h
 * @brief 为电机生产控制测试提供只读、类型固定的标量快照来源。
 * 使用条件：仅主机测试使用，context 指向测试生命周期内的 float；样本固定从时刻0开始。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_MOTOR_FEEDBACK_TEST_SUPPORT_H
#define MC02_MOTOR_FEEDBACK_TEST_SUPPORT_H

#include "motor_feedback.h"

#include <stddef.h>

static inline Motor_Feedback_Status_e MotorFeedbackTestReadAs(
    const void *context,
    Motor_Signal_Snapshot_s *snapshot,
    Motor_Unit_e unit,
    Motor_Reference_e reference)
{
    if (context == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    *snapshot = (Motor_Signal_Snapshot_s){
        .value = *(const float *)context,
        .unit = unit,
        .reference = reference,
        .source_time_ms = 0u,
        .sequence = 1u,
        .valid = 1u,
    };
    return MOTOR_FEEDBACK_OK;
}

#define MOTOR_FEEDBACK_TEST_READER(name, unit_value, reference_value) \
    static inline Motor_Feedback_Status_e name(const void *context, Motor_Signal_Snapshot_s *snapshot) \
    { \
        return MotorFeedbackTestReadAs(context, snapshot, unit_value, reference_value); \
    }

MOTOR_FEEDBACK_TEST_READER(MotorFeedbackTestReadRadExternal, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL)
MOTOR_FEEDBACK_TEST_READER(MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL)
MOTOR_FEEDBACK_TEST_READER(MotorFeedbackTestReadRadSOutput, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS)
MOTOR_FEEDBACK_TEST_READER(MotorFeedbackTestReadDeviceCurrentOutput,
                           MOTOR_UNIT_DEVICE_CURRENT, MOTOR_REFERENCE_OUTPUT_AXIS)
MOTOR_FEEDBACK_TEST_READER(MotorFeedbackTestReadDeviceTorqueOutput,
                           MOTOR_UNIT_DEVICE_TORQUE, MOTOR_REFERENCE_OUTPUT_AXIS)

static inline Motor_Signal_Source_s MotorFeedbackTestMakeSource(
    const float *value,
    Motor_Signal_Read_Callback reader,
    Motor_Unit_e unit,
    Motor_Reference_e reference)
{
    if (value == NULL || reader == NULL)
        return (Motor_Signal_Source_s){0};
    return (Motor_Signal_Source_s){
        .read = reader,
        .context = value,
        .max_age_ms = (uint32_t)INT32_MAX,
        .unit = unit,
        .reference = reference,
    };
}

#endif /* MC02_MOTOR_FEEDBACK_TEST_SUPPORT_H */
