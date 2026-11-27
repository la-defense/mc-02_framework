/** @file dji_motor_control_config.c
 * @brief 车型预检和 DJI 电机登记共用的控制来源检查。
 */
#include "dji_motor_control_config.h"
#include <stddef.h>

// 公共接口验证回调和期限，DJI 控制环另有明确的单位与参考系要求。
static uint8_t FeedbackSourceValid(const Motor_Signal_Source_s *source,
                                   Motor_Unit_e unit, Motor_Reference_e reference)
{
    return (uint8_t)(MotorFeedbackValidateSignalSource(source) == MOTOR_FEEDBACK_OK &&
                     source->unit == unit && source->reference == reference);
}

uint8_t DJIMotorControlConfigIsValid(const Motor_Init_Config_s *config)
{
    if (config == NULL) return 0u; // 无配置时不能读取控制环声明。
    const Motor_Control_Setting_s *setting = &config->controller_setting_init_config; // 声明启用哪些来源。
    const Motor_Controller_Init_s *controller = &config->controller_param_init_config; // 保存实际回调及单位。
    if (setting->motor_reverse_flag != MOTOR_DIRECTION_NORMAL ||
        setting->feedback_reverse_flag != FEEDBACK_DIRECTION_NORMAL ||
        (setting->feedforward_flag & (uint8_t)~CURRENT_AND_SPEED_FEEDFORWARD) != 0u)
        return 0u; // 安装方向已由输出轴配置表达，前馈只接受已支持的选项。
    if ((setting->angle_feedback_source != MOTOR_FEED && setting->angle_feedback_source != OTHER_FEED) ||
        (setting->speed_feedback_source != MOTOR_FEED && setting->speed_feedback_source != OTHER_FEED))
        return 0u; // 即使有些环暂未启用，来源枚举也须是已知选项。

    if ((setting->close_loop_type & ANGLE_LOOP) != 0u && setting->outer_loop_type == ANGLE_LOOP &&
        setting->angle_feedback_source == OTHER_FEED &&
        !FeedbackSourceValid(&controller->other_angle_feedback, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL))
        return 0u; // 启用的外部角度来源须提供以 rad 表达的数据。
    if ((setting->close_loop_type & SPEED_LOOP) != 0u &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) != 0u &&
        setting->speed_feedback_source == OTHER_FEED &&
        !FeedbackSourceValid(&controller->other_speed_feedback, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL))
        return 0u; // 启用的外部速度来源须提供以 rad/s 表达的数据。
    if ((setting->feedforward_flag & SPEED_FEEDFORWARD) != 0u &&
        !FeedbackSourceValid(&controller->speed_feedforward, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_OUTPUT_AXIS))
        return 0u; // 速度前馈使用机械输出轴速度。
    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) != 0u &&
        !FeedbackSourceValid(&controller->current_feedforward, MOTOR_UNIT_DEVICE_CURRENT, MOTOR_REFERENCE_OUTPUT_AXIS))
        return 0u; // 电流前馈使用 DJI 设备电流单位，方向与输出轴对应。
    return 1u; // 只声明未启用的信号时，无需提供读取回调。
}
