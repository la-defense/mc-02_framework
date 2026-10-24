/**
 * @file motor_feedback.h
 * @brief 声明电机能力、输出轴换算配置及统一测量单位。
 * 入口：MotorFeedbackGetCapabilities() 查询驱动能力，MotorFeedbackConvertOutputAxis() 换算转子测量值。
 * 使用条件：单位换算不读取硬件时钟、不分配内存；输出轴减速比和方向须由应用明确配置。
 * @author MC-02 contributors
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 */
#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include <stdint.h>

/** @brief 固件中登记的电机类型；未实现的类型仍会被能力查询拒绝。 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
    DR04,
    XMCY,
} Motor_Type_e;

/** @brief 统一测量值使用的物理单位；设备原始数值不冒充 SI 单位。 */
typedef enum
{
    MOTOR_UNIT_UNSPECIFIED = 0,
    MOTOR_UNIT_RAD,
    MOTOR_UNIT_RAD_S,
    MOTOR_UNIT_DEG,
    MOTOR_UNIT_DEG_S,
    MOTOR_UNIT_AMPERE,
    MOTOR_UNIT_NEWTON_METER,
    MOTOR_UNIT_VOLT,
    MOTOR_UNIT_DEVICE_CURRENT,
    MOTOR_UNIT_DEVICE_TORQUE,
} Motor_Unit_e;

/** @brief 测量值对应的机械参考位置。 */
typedef enum
{
    MOTOR_REFERENCE_UNSPECIFIED = 0,
    MOTOR_REFERENCE_OUTPUT_AXIS,
    MOTOR_REFERENCE_ROTOR,
    MOTOR_REFERENCE_EXTERNAL,
    MOTOR_REFERENCE_DEVICE,
} Motor_Reference_e;

/** @brief 单个外部反馈或前馈值的只读来源快照。 */
typedef struct
{
    float value; /**< 有限数值；单位由 unit 指定。 */
    Motor_Unit_e unit; /**< 不允许 MOTOR_UNIT_UNSPECIFIED。 */
    Motor_Reference_e reference; /**< 不允许 MOTOR_REFERENCE_UNSPECIFIED。 */
    uint32_t source_time_ms; /**< 来源采样时刻；0 是合法时刻。 */
    uint32_t sequence; /**< 来源序号；0 保留为尚无样本。 */
    uint8_t valid; /**< 仅值为1时可供控制读取。 */
} Motor_Signal_Snapshot_s;

/** @brief 同一来源时刻复制的电机反馈集合；不支持的字段保持 valid=0。 */
typedef struct
{
    uint32_t sample_sequence; /**< 本次设备反馈序号；0 表示尚无有效样本。 */
    Motor_Signal_Snapshot_s position; /**< 输出轴位置，rad。 */
    Motor_Signal_Snapshot_s speed; /**< 输出轴速度，rad/s。 */
    Motor_Signal_Snapshot_s current; /**< 设备电流量纲，不保证为A。 */
    Motor_Signal_Snapshot_s torque; /**< 设备力矩数值，不保证为N·m。 */
    Motor_Signal_Snapshot_s voltage; /**< 设备电压，V；驱动不支持时无效。 */
} Motor_Feedback_Snapshot_s;

/** @brief 当前驱动可提供的测量字段。 */
typedef enum
{
    MOTOR_CAPABILITY_NONE = 0u,
    MOTOR_CAPABILITY_POSITION = 1u << 0,
    MOTOR_CAPABILITY_SPEED = 1u << 1,
    MOTOR_CAPABILITY_CURRENT = 1u << 2,
    MOTOR_CAPABILITY_TORQUE = 1u << 3,
    MOTOR_CAPABILITY_VOLTAGE = 1u << 4,
} Motor_Capability_e;

/** @brief 输出轴相对转子的安装方向。 */
typedef enum
{
    MOTOR_AXIS_DIRECTION_REVERSE = -1,
    MOTOR_AXIS_DIRECTION_NORMAL = 1,
} Motor_Axis_Direction_e;

/** @brief 电机驱动支持的反馈字段及其单位。 */
typedef struct
{
    uint32_t measurement_mask; /**< MOTOR_CAPABILITY_* 位掩码。 */
    Motor_Unit_e position_unit; /**< 支持位置时固定为 rad。 */
    Motor_Unit_e speed_unit; /**< 支持速度时固定为 rad/s。 */
    Motor_Unit_e current_unit; /**< 不支持时为 UNSPECIFIED。 */
    Motor_Unit_e torque_unit; /**< 不支持时为 UNSPECIFIED。 */
    Motor_Unit_e voltage_unit; /**< 不支持时为 UNSPECIFIED。 */
} Motor_Capabilities_s;

/** @brief 转子到输出轴的明确换算设置，减速比定义为转子转数／输出轴转数。 */
typedef struct
{
    float gear_ratio; /**< 有限正数，允许范围 (0, FLT_MAX]。 */
    Motor_Axis_Direction_e direction; /**< 正常方向为 +1，反向为 -1。 */
} Motor_Output_Axis_Config_s;

/** @brief 纯数值模块的状态。 */
typedef enum
{
    MOTOR_FEEDBACK_OK = 0,
    MOTOR_FEEDBACK_BAD_ARGUMENT,
    MOTOR_FEEDBACK_BAD_CONFIG,
    MOTOR_FEEDBACK_UNSUPPORTED,
    MOTOR_FEEDBACK_NONFINITE,
    MOTOR_FEEDBACK_OVERFLOW,
    MOTOR_FEEDBACK_INVALID_SNAPSHOT,
    MOTOR_FEEDBACK_STALE,
    MOTOR_FEEDBACK_UNIT_MISMATCH,
    MOTOR_FEEDBACK_REFERENCE_MISMATCH,
    MOTOR_FEEDBACK_SOURCE_ERROR,
    MOTOR_FEEDBACK_NOT_READY,
} Motor_Feedback_Status_e;

/**
 * @brief 复制一个完整外部信号快照的回调。
 * @param[in] context 调用方长期保存的只读上下文。
 * @param[out] snapshot 候选输出；回调成功时必须一次写入完整快照。
 * @return 成功返回 MOTOR_FEEDBACK_OK；其他值表示来源读取失败。
 */
typedef Motor_Feedback_Status_e (*Motor_Signal_Read_Callback)(const void *context,
                                                               Motor_Signal_Snapshot_s *snapshot);

/** @brief 外部反馈或前馈的只读来源；上下文须比使用它的电机实例存活更久。 */
typedef struct
{
    Motor_Signal_Read_Callback read; /**< 每次控制周期最多调用一次。 */
    const void *context; /**< 由调用方保存，读取回调不得返回内部可写地址。 */
    uint32_t max_age_ms; /**< 来源最大年龄，1～INT32_MAX ms；超出即拒绝本周期。 */
    Motor_Unit_e unit; /**< 来源配置声明的单位，更新快照必须与之相同。 */
    Motor_Reference_e reference; /**< 来源配置声明的参考系，更新快照必须与之相同。 */
} Motor_Signal_Source_s;

/**
 * @brief 查询指定驱动可提供的测量字段和单位。
 * @param[in] motor_type 已登记的电机类型。
 * @param[out] capabilities 输出驱动能力；成功时覆盖，失败时保持原值。
 * @return MOTOR_FEEDBACK_OK 表示成功；未知或尚未实现的类型返回 UNSUPPORTED。
 */
Motor_Feedback_Status_e MotorFeedbackGetCapabilities(Motor_Type_e motor_type,
                                                      Motor_Capabilities_s *capabilities);

/**
 * @brief 校验输出轴减速比和方向配置。
 * @param[in] config 输出轴配置，gear_ratio 必须有限且大于0，direction 必须为 ±1。
 * @return MOTOR_FEEDBACK_OK 表示可用；空参数、非有限值或非法范围返回具体状态。
 */
Motor_Feedback_Status_e MotorFeedbackValidateOutputAxis(const Motor_Output_Axis_Config_s *config);

/**
 * @brief 将转子位置和速度换算为输出轴 rad 与 rad/s。
 * @param[in] config 已校验的输出轴减速比与方向。
 * @param[in] rotor_position_rad 转子位置，单位 rad。
 * @param[in] rotor_speed_rad_s 转子速度，单位 rad/s。
 * @param[out] output_position_rad 输出轴位置；失败时保持原值。
 * @param[out] output_speed_rad_s 输出轴速度；失败时保持原值。
 * @return MOTOR_FEEDBACK_OK 表示成功；失败时两个输出均保持原值。
 */
Motor_Feedback_Status_e MotorFeedbackConvertOutputAxis(const Motor_Output_Axis_Config_s *config,
                                                       float rotor_position_rad,
                                                       float rotor_speed_rad_s,
                                                       float *output_position_rad,
                                                       float *output_speed_rad_s);

/**
 * @brief 将转子角度制位置和速度换算为输出轴 rad 与 rad/s。
 * @param[in] config 输出轴减速比与方向。
 * @param[in] rotor_position_deg 转子位置，单位 deg。
 * @param[in] rotor_speed_deg_s 转子速度，单位 deg/s。
 * @param[out] output_position_rad 输出轴位置；失败时保持原值。
 * @param[out] output_speed_rad_s 输出轴速度；失败时保持原值。
 * @return MOTOR_FEEDBACK_OK 表示成功；失败时两个输出均保持原值。
 */
Motor_Feedback_Status_e MotorFeedbackConvertDegreesToOutputAxis(const Motor_Output_Axis_Config_s *config,
                                                                float rotor_position_deg,
                                                                float rotor_speed_deg_s,
                                                                float *output_position_rad,
                                                                float *output_speed_rad_s);

/** @brief 检查外部来源在本轮实际读取前是否配置完整。 */
Motor_Feedback_Status_e MotorFeedbackValidateSignalSource(const Motor_Signal_Source_s *source);

/**
 * @brief 读取并验证一次外部信号快照，输出只在全部检查通过后更新。
 * @param[in] source 来源回调与其长期只读上下文。
 * @param[in] expected_unit 本次控制分支要求的单位，不能为 UNSPECIFIED。
 * @param[in] expected_reference 本次控制分支要求的参考系，不能为 UNSPECIFIED。
 * @param[in] now_ms 调用方提供的当前毫秒时刻；允许0和无符号回绕。
 * @param[out] output 成功时复制完整快照；失败时保持原值。
 * @return 来源读取、数据有效、有限性、单位、参考系或新鲜度对应的状态。
 * @note 回调须自行使用适当同步，一次复制一致数据；本函数不读硬件时钟、不分配内存。
 */
Motor_Feedback_Status_e MotorFeedbackReadSignal(const Motor_Signal_Source_s *source,
                                                 Motor_Unit_e expected_unit,
                                                 Motor_Reference_e expected_reference,
                                                 uint32_t now_ms,
                                                 Motor_Signal_Snapshot_s *output);

#endif /* MOTOR_FEEDBACK_H */
