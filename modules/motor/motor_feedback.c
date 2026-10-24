/**
 * @file motor_feedback.c
 * @brief 查询电机测量能力，并进行输出轴单位换算。
 * 入口：MotorFeedbackGetCapabilities()、MotorFeedbackValidateOutputAxis()、MotorFeedbackConvertOutputAxis()。
 * 使用条件：纯 C 数值处理；不访问 CAN、RTOS 或动态分配。
 * @author MC-02 contributors
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "motor_feedback.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

Motor_Feedback_Status_e MotorFeedbackGetCapabilities(Motor_Type_e motor_type,
                                                      Motor_Capabilities_s *capabilities)
{
    if (capabilities == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    Motor_Capabilities_s candidate = {0};
    switch (motor_type)
    {
    case GM6020:
    case M3508:
    case M2006:
        candidate.measurement_mask = MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED |
                                     MOTOR_CAPABILITY_CURRENT;
        candidate.position_unit = MOTOR_UNIT_RAD;
        candidate.speed_unit = MOTOR_UNIT_RAD_S;
        candidate.current_unit = MOTOR_UNIT_DEVICE_CURRENT;
        break;
    case LK9025:
        candidate.measurement_mask = MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED |
                                     MOTOR_CAPABILITY_CURRENT;
        candidate.position_unit = MOTOR_UNIT_RAD;
        candidate.speed_unit = MOTOR_UNIT_RAD_S;
        candidate.current_unit = MOTOR_UNIT_DEVICE_CURRENT;
        break;
    case HT04:
        candidate.measurement_mask = MOTOR_CAPABILITY_POSITION | MOTOR_CAPABILITY_SPEED |
                                     MOTOR_CAPABILITY_TORQUE;
        candidate.position_unit = MOTOR_UNIT_RAD;
        candidate.speed_unit = MOTOR_UNIT_RAD_S;
        candidate.torque_unit = MOTOR_UNIT_DEVICE_TORQUE;
        break;
    default:
        return MOTOR_FEEDBACK_UNSUPPORTED;
    }

    *capabilities = candidate;
    return MOTOR_FEEDBACK_OK;
}

Motor_Feedback_Status_e MotorFeedbackValidateOutputAxis(const Motor_Output_Axis_Config_s *config)
{
    if (config == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    if (!isfinite(config->gear_ratio))
        return MOTOR_FEEDBACK_NONFINITE;
    if (config->gear_ratio <= 0.0f ||
        (config->direction != MOTOR_AXIS_DIRECTION_NORMAL &&
         config->direction != MOTOR_AXIS_DIRECTION_REVERSE))
        return MOTOR_FEEDBACK_BAD_CONFIG;
    return MOTOR_FEEDBACK_OK;
}

static uint8_t ResultFitsFloat(double value)
{
    return (uint8_t)(isfinite(value) && value <= (double)FLT_MAX && value >= -(double)FLT_MAX);
}

static uint8_t UnitIsKnown(Motor_Unit_e unit)
{
    return (uint8_t)(unit >= MOTOR_UNIT_RAD && unit <= MOTOR_UNIT_DEVICE_TORQUE);
}

static uint8_t ReferenceIsKnown(Motor_Reference_e reference)
{
    return (uint8_t)(reference >= MOTOR_REFERENCE_OUTPUT_AXIS &&
                     reference <= MOTOR_REFERENCE_DEVICE);
}

Motor_Feedback_Status_e MotorFeedbackConvertOutputAxis(const Motor_Output_Axis_Config_s *config,
                                                       float rotor_position_rad,
                                                       float rotor_speed_rad_s,
                                                       float *output_position_rad,
                                                       float *output_speed_rad_s)
{
    if (config == NULL || output_position_rad == NULL || output_speed_rad_s == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    Motor_Feedback_Status_e status = MotorFeedbackValidateOutputAxis(config);
    if (status != MOTOR_FEEDBACK_OK)
        return status;
    if (!isfinite(rotor_position_rad) || !isfinite(rotor_speed_rad_s))
        return MOTOR_FEEDBACK_NONFINITE;

    const double direction = (double)config->direction;
    const double ratio = (double)config->gear_ratio;
    const double candidate_position = direction * (double)rotor_position_rad / ratio;
    const double candidate_speed = direction * (double)rotor_speed_rad_s / ratio;
    if (!ResultFitsFloat(candidate_position) || !ResultFitsFloat(candidate_speed))
        return MOTOR_FEEDBACK_OVERFLOW;

    // 两个结果都通过范围检查后再写出，避免部分更新。
    *output_position_rad = (float)candidate_position;
    *output_speed_rad_s = (float)candidate_speed;
    return MOTOR_FEEDBACK_OK;
}

Motor_Feedback_Status_e MotorFeedbackConvertDegreesToOutputAxis(const Motor_Output_Axis_Config_s *config,
                                                                float rotor_position_deg,
                                                                float rotor_speed_deg_s,
                                                                float *output_position_rad,
                                                                float *output_speed_rad_s)
{
    if (config == NULL || output_position_rad == NULL || output_speed_rad_s == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    if (!isfinite(rotor_position_deg) || !isfinite(rotor_speed_deg_s))
        return MOTOR_FEEDBACK_NONFINITE;

    const double radians_per_degree = 0.017453292519943295769236907684886;
    const float rotor_position_rad = (float)((double)rotor_position_deg * radians_per_degree);
    const float rotor_speed_rad_s = (float)((double)rotor_speed_deg_s * radians_per_degree);
    if (!isfinite(rotor_position_rad) || !isfinite(rotor_speed_rad_s))
        return MOTOR_FEEDBACK_OVERFLOW;
    return MotorFeedbackConvertOutputAxis(config, rotor_position_rad, rotor_speed_rad_s,
                                          output_position_rad, output_speed_rad_s);
}

Motor_Feedback_Status_e MotorFeedbackValidateSignalSource(const Motor_Signal_Source_s *source)
{
    if (source == NULL || source->read == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    if (source->max_age_ms == 0u || source->max_age_ms > (uint32_t)INT32_MAX)
        return MOTOR_FEEDBACK_BAD_CONFIG;
    if (!UnitIsKnown(source->unit) || !ReferenceIsKnown(source->reference))
        return MOTOR_FEEDBACK_BAD_CONFIG;
    return MOTOR_FEEDBACK_OK;
}

Motor_Feedback_Status_e MotorFeedbackReadSignal(const Motor_Signal_Source_s *source,
                                                 Motor_Unit_e expected_unit,
                                                 Motor_Reference_e expected_reference,
                                                 uint32_t now_ms,
                                                 Motor_Signal_Snapshot_s *output)
{
    if (output == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    Motor_Feedback_Status_e source_status = MotorFeedbackValidateSignalSource(source);
    if (source_status != MOTOR_FEEDBACK_OK)
        return source_status;
    if (!UnitIsKnown(expected_unit) || !ReferenceIsKnown(expected_reference))
        return MOTOR_FEEDBACK_BAD_CONFIG;
    if (source->unit != expected_unit || source->reference != expected_reference)
        return MOTOR_FEEDBACK_UNIT_MISMATCH;

    Motor_Signal_Snapshot_s candidate = {0};
    if (source->read(source->context, &candidate) != MOTOR_FEEDBACK_OK)
        return MOTOR_FEEDBACK_SOURCE_ERROR;
    if (candidate.valid != 1u || candidate.sequence == 0u || !UnitIsKnown(candidate.unit) ||
        !ReferenceIsKnown(candidate.reference))
        return MOTOR_FEEDBACK_INVALID_SNAPSHOT;
    if (!isfinite(candidate.value))
        return MOTOR_FEEDBACK_NONFINITE;
    if (candidate.unit != expected_unit)
        return MOTOR_FEEDBACK_UNIT_MISMATCH;
    if (candidate.reference != expected_reference)
        return MOTOR_FEEDBACK_REFERENCE_MISMATCH;
    if (candidate.unit != source->unit || candidate.reference != source->reference)
        return MOTOR_FEEDBACK_INVALID_SNAPSHOT;

    // 无符号减法同时覆盖零时刻和计时器回绕；最大年龄限制在半个计数周期内。
    const uint32_t age_ms = now_ms - candidate.source_time_ms;
    if (age_ms > source->max_age_ms)
        return MOTOR_FEEDBACK_STALE;

    *output = candidate;
    return MOTOR_FEEDBACK_OK;
}
