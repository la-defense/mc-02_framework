/**
 * @file robot_def.c
 * @brief 选择唯一车型配置并提供只读分项访问入口。
 */
#include "config.h"
#include "sentinel_config.h"
#include "../modules/general_def.h"

#if MC02_FEATURE_CHASSIS
#if MC02_CHASSIS_IS_OMNI
#include "omni.h"
#else
#include "mecanum.h"
#endif
#endif

#include <math.h>
#include <stddef.h>

static const Robot_Config_s *robot_instance;

#if MC02_FEATURE_CHASSIS || MC02_FEATURE_GIMBAL || MC02_FEATURE_SHOOTER
// 检查公共总线、减速比和驱动型号，不读取需要按实例填入的方向和编号。
static uint8_t DJIHardwareConfigIsValid(const Motor_Init_Config_s *motor)
{
    return motor->can_init_config.can_bus >= PLATFORM_CAN_BUS_1 && // 识别工程已声明的逻辑 CAN 总线。
        motor->can_init_config.can_bus <= PLATFORM_CAN_BUS_3 && // 超出平台总线范围时拒绝。
        motor->can_init_config.identifier_type == CAN_TX_ID_STANDARD && // DJI 设备使用标准 ID。
        isfinite(motor->output_axis.gear_ratio) && motor->output_axis.gear_ratio > 0.0f && // 杜绝非法换算比例。
        (motor->motor_type == GM6020 || motor->motor_type == M3508 || motor->motor_type == M2006); // 这三种型号已有 DJI 驱动。
}

// 安装方向须显式给出，零表示待确认。
static uint8_t DJIOutputDirectionIsValid(Motor_Axis_Direction_e direction)
{
    return direction == MOTOR_AXIS_DIRECTION_NORMAL || direction == MOTOR_AXIS_DIRECTION_REVERSE;
}
#endif

#if MC02_FEATURE_GIMBAL || MC02_FEATURE_SHOOTER
// 其他机构的安装方向仍从完整电机配置取得。
static uint8_t IsSupportedDJIMotor(const Motor_Init_Config_s *motor, uint8_t require_id)
{
    return DJIHardwareConfigIsValid(motor) && DJIOutputDirectionIsValid(motor->output_axis.direction) &&
        (require_id == 0u || (motor->can_init_config.tx_id >= 1u && motor->can_init_config.tx_id <= 8u));
}
#endif

#if MC02_FEATURE_CHASSIS
// 确认底盘尺寸、轮电机编号和已编译运动学模型能够共同计算有效轮速。
static uint8_t ChassisConfigIsValid(const Chassis_Config_s *chassis)
{
    if (chassis->motion_limits.configured == 0u ||
        !isfinite(chassis->motion_limits.max_wz_deg_s) ||
        chassis->motion_limits.max_wz_deg_s <= 0.0f ||
        !isfinite(chassis->motion_limits.rotate_wz_deg_s) ||
        fabsf(chassis->motion_limits.rotate_wz_deg_s) > chassis->motion_limits.max_wz_deg_s ||
        !isfinite(chassis->motion_limits.max_motor_speed_deg_s) ||
        chassis->motion_limits.max_motor_speed_deg_s <= 0.0f ||
        !DJIHardwareConfigIsValid(&chassis->chassis_motor_config))
        return 0u;

    if ((chassis->wheel_type != MECANUM_WHEEL && chassis->wheel_type != OMNI_WHEEL) ||
        !ChassisOdometryGapIsValid(chassis->odometry_max_gap_ms, chassis->wheel_type == OMNI_WHEEL))
        return 0u;
#if defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    const uint16_t output_limit = chassis->chassis_motor_config.dji_chassis_output_limit_raw;
    if (output_limit == 0u || output_limit > DJI_OUTPUT_MAX_RAW)
        return 0u; // 运动固件须提供明确的轮组输出上限，不能沿用驱动默认范围。

    const float reduction = chassis->chassis_motor_config.output_axis.gear_ratio; // 轮轴和电机轴共用车型声明的减速比。
    const float motor_deg_per_wheel_rad = RAD_2_DEGREE * reduction; // 按控制周期的算法将轮轴 rad/s 换成电机轴 deg/s。
    const float wheel_rad_per_motor_deg = DEGREE_2_RAD / reduction; // 驱动接收轮轴 rad/s，按同一减速比换回。
    const float max_wheel_rad_s = chassis->motion_limits.max_motor_speed_deg_s * wheel_rad_per_motor_deg; // 核对换回轮轴后的有效上限。
    if (!isfinite(motor_deg_per_wheel_rad) || motor_deg_per_wheel_rad <= 0.0f ||
        !isfinite(max_wheel_rad_s) || max_wheel_rad_s <= 0.0f)
        return 0u; // 系数溢出、上限溢出或上限被舍入为零时拒绝启动。

    const ChassisTwist_t max_spin = {
        .wz_rad_s = chassis->motion_limits.max_wz_deg_s * DEGREE_2_RAD, // 用配置的最大旋转速度检查从底盘到轮轴的换算。
    };
    if (max_spin.wz_rad_s <= 0.0f)
        return 0u; // 配置给出的正角速度换算后必须仍可表示。

    ChassisWheelSpeeds_t wheel_speeds; // 由已编译的模型按固定轮序返回轮轴 rad/s。
#if MC02_CHASSIS_IS_OMNI
    if (chassis->wheel_type != OMNI_WHEEL ||
        OmniCalculate(&max_spin, &chassis->omni_geometry, &wheel_speeds) !=
            CHASSIS_KINEMATICS_OK)
        return 0u; // 拒绝全向轮几何不可用或该转速无法表示的配置。
#else
    if (chassis->wheel_type != MECANUM_WHEEL ||
        MecanumCalculate(&max_spin, &chassis->mecanum_geometry, &wheel_speeds) !=
            CHASSIS_KINEMATICS_OK)
        return 0u; // 拒绝麦轮几何不可用或该转速无法表示的配置。
#endif
    for (unsigned index = 0u; index < CHASSIS_WHEEL_COUNT; ++index) // 逐轮核对限速前的电机轴转速。
    {
        const float rotor_speed_deg_s = wheel_speeds.rad_s[index] * motor_deg_per_wheel_rad; // 旧接口先换算再按比例限速。
        if (!isfinite(rotor_speed_deg_s))
            return 0u; // 换算已溢出时，限速无法恢复有效轮速。
    }
#endif
    // 四台轮电机共用型号和总线，各自声明编号与安装方向。
    for (unsigned index = 0u; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        const ChassisWheelMotorConfig_t *wheel = &chassis->wheel_motors[index]; // 按轮序读取同一份设备配置。
        if (wheel->device_id < 1u || wheel->device_id > 8u || !DJIOutputDirectionIsValid(wheel->direction))
            return 0u; // 未确认方向或非法编号不能启动轮组。
        for (unsigned previous = 0u; previous < index; ++previous) // 同一总线不能把同一设备分配给两轮。
            if (chassis->wheel_motors[previous].device_id == wheel->device_id)
                return 0u; // 两轮收到相同设备反馈会使速度计算无效。
    }
    return 1u;
}
#endif

#if MC02_FEATURE_GIMBAL
// 校验当前两轴云台的角度范围、电机能力和 CAN 节点唯一性。
static uint8_t GimbalConfigIsValid(const Gimbal_Config_s *gimbal)
{
    if (gimbal->gimbal_type != SINGLE_GIMBAL ||
        !isfinite(gimbal->gimbal_offset.yaw_offset) ||
        gimbal->gimbal_offset.yaw_offset < 0.0f ||
        gimbal->gimbal_offset.yaw_offset >= 360.0f ||
        !isfinite(gimbal->gimbal_offset.pitch_min_angle) ||
        !isfinite(gimbal->gimbal_offset.pitch_max_angle) ||
        gimbal->gimbal_offset.pitch_min_angle >= gimbal->gimbal_offset.pitch_max_angle ||
        !IsSupportedDJIMotor(&gimbal->yaw_motor_config, 1u) ||
        !IsSupportedDJIMotor(&gimbal->pitch_motor_config, 1u))
        return 0u;
    if (gimbal->yaw_motor_config.can_init_config.can_bus ==
            gimbal->pitch_motor_config.can_init_config.can_bus &&
        gimbal->yaw_motor_config.can_init_config.tx_id ==
            gimbal->pitch_motor_config.can_init_config.tx_id)
        return 0u;
    return 1u;
}
#endif

#if MC02_FEATURE_SHOOTER
// 校验拨弹与摩擦轮参数，并由发射机构类型确定本车启用的摩擦轮数。
static uint8_t ShootConfigIsValid(const Shoot_Config_s *shoot)
{
    const uint8_t friction_count = ShootFrictionWheelCount(shoot->shoot_type);
    if (friction_count == 0u) return 0u;
    if (shoot->loader_slot_num == 0u ||
        !isfinite(shoot->loader_gearbox_ratio) || shoot->loader_gearbox_ratio <= 0.0f ||
        !isfinite(shoot->loader_disk_ratio) || shoot->loader_disk_ratio <= 0.0f ||
        !isfinite(shoot->friction_default_speed_rad_s) ||
        shoot->friction_default_speed_rad_s <= 0.0f ||
        !isfinite(shoot->loader_reverse_speed_rad_s) || shoot->loader_reverse_speed_rad_s <= 0.0f ||
        !IsSupportedDJIMotor(&shoot->friction_motor_config, 0u) ||
        !IsSupportedDJIMotor(&shoot->loader_motor_config, 1u))
        return 0u;
    // 检查启用摩擦轮的编号、方向及与拨弹电机的冲突。
    for (uint8_t index = 0u; index < friction_count; ++index)
    {
        if (shoot->friction_motor_id[index] == 0u || shoot->friction_motor_id[index] > 8u ||
            (shoot->friction_motor_reverse_flag[index] != MOTOR_DIRECTION_NORMAL &&
             shoot->friction_motor_reverse_flag[index] != MOTOR_DIRECTION_REVERSE))
            return 0u;
        if (shoot->friction_motor_config.can_init_config.can_bus ==
                shoot->loader_motor_config.can_init_config.can_bus &&
            shoot->friction_motor_id[index] ==
                shoot->loader_motor_config.can_init_config.tx_id)
            return 0u;
        for (uint8_t previous = 0u; previous < index; ++previous)
            if (shoot->friction_motor_id[previous] == shoot->friction_motor_id[index])
                return 0u;
    }
    return 1u;
}
#endif

// 按本固件启用的业务能力校验对应配置，未启用的功能不读取其参数。
static uint8_t RobotConfigParametersAreValid(const Robot_Config_s *config)
{
    if (config == NULL)
        return 0u;
#if MC02_FEATURE_CHASSIS
    if (!ChassisConfigIsValid(&config->chassis_param))
        return 0u;
#endif
#if MC02_FEATURE_GIMBAL
    if (!GimbalConfigIsValid(&config->gimbal_param))
        return 0u;
#endif
#if MC02_FEATURE_SHOOTER
    if (!ShootConfigIsValid(&config->shoot_param))
        return 0u;
#endif
    return 1u;
}

RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output)
{
    if (output == NULL)
        return ROBOT_CONFIG_ARGUMENT;

    *output = NULL;
    robot_instance = NULL;
    if (id != SENTINEL_ROBOT)
        return ROBOT_CONFIG_UNSUPPORTED;

    const Robot_Config_s *candidate = SentinelConfigInit();
    if (!RobotConfigParametersAreValid(candidate))
        return ROBOT_CONFIG_BAD_PARAMETERS;
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    if (!RobotCommissioningChassisInputIsValid(&candidate->manual_input))
        return ROBOT_CONFIG_BAD_MANUAL_INPUT; // 在设备和受控电源注册前拒绝错误的人工运动参数。
#endif

    robot_instance = candidate;
    *output = candidate;
    return ROBOT_CONFIG_OK;
}

const Robot_Config_s *RobotConfigInit(uint8_t id)
{
    const Robot_Config_s *config = NULL;
    return RobotConfigSelect(id, &config) == ROBOT_CONFIG_OK ? config : NULL;
}

const Robot_Config_s *RobotConfigFeed(void)
{
    return robot_instance;
}

const Chassis_Config_s *ChassisConfigFeed(void)
{
#if MC02_FEATURE_CHASSIS
    return robot_instance != NULL ? &robot_instance->chassis_param : NULL;
#else
    return NULL;
#endif
}

const Gimbal_Config_s *GimbalConfigFeed(void)
{
#if MC02_FEATURE_GIMBAL
    return robot_instance != NULL ? &robot_instance->gimbal_param : NULL;
#else
    return NULL;
#endif
}

const Shoot_Config_s *ShootConfigFeed(void)
{
#if MC02_FEATURE_SHOOTER
    return robot_instance != NULL ? &robot_instance->shoot_param : NULL;
#else
    return NULL;
#endif
}

const RefereeUIConfig_t *RefereeUIConfigFeed(void)
{
#if MC02_FEATURE_UI
    return robot_instance != NULL ? robot_instance->referee_ui_config : NULL;
#else
    return NULL;
#endif
}
