/**
 * @file robot_def.c
 * @brief 选择唯一车型配置并提供只读分项访问入口。
 */
#include "config.h"
#include "sentinel_config.h"

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

#if MC02_FEATURE_CHASSIS || MC02_FEATURE_GIMBAL || MC02_FEATURE_SHOOTER || \
    defined(MC02_PROFILE_ROBOT)
// 校验配置中的 DJI 电机类型、安装方向和本场景要求的发送 ID。
static uint8_t IsSupportedDJIMotor(const Motor_Init_Config_s *motor, uint8_t require_id)
{
    if (motor == NULL || motor->can_init_config.can_bus < PLATFORM_CAN_BUS_1 ||
        motor->can_init_config.can_bus > PLATFORM_CAN_BUS_3 ||
        !isfinite(motor->output_axis.gear_ratio) || motor->output_axis.gear_ratio <= 0.0f ||
        (motor->output_axis.direction != MOTOR_AXIS_DIRECTION_NORMAL &&
         motor->output_axis.direction != MOTOR_AXIS_DIRECTION_REVERSE) ||
        (motor->motor_type != GM6020 && motor->motor_type != M3508 && motor->motor_type != M2006))
        return 0u;
    if (require_id != 0u &&
        (motor->can_init_config.tx_id == 0u || motor->can_init_config.tx_id > 8u))
        return 0u;
    return 1u;
}
#endif

#if MC02_FEATURE_CHASSIS
// 确认底盘尺寸、轮电机编号和已编译运动学模型能够共同计算有效轮速。
static uint8_t ChassisConfigIsValid(const Chassis_Config_s *chassis)
{
    const Wheel_Measure_s *measure = &chassis->wheel_measure;
    if (!isfinite(measure->wheel_base) || measure->wheel_base <= 0.0f ||
        !isfinite(measure->track_width) || measure->track_width <= 0.0f ||
        !isfinite(measure->radius_wheel) || measure->radius_wheel <= 0.0f ||
        !isfinite(measure->reduction_ratio_wheel) || measure->reduction_ratio_wheel <= 0.0f ||
        !isfinite(measure->center_gimbal_offset_x) ||
        !isfinite(measure->center_gimbal_offset_y) ||
        chassis->motion_limits.configured == 0u ||
        !isfinite(chassis->motion_limits.max_wz_deg_s) ||
        chassis->motion_limits.max_wz_deg_s <= 0.0f ||
        !isfinite(chassis->motion_limits.rotate_wz_deg_s) ||
        fabsf(chassis->motion_limits.rotate_wz_deg_s) > chassis->motion_limits.max_wz_deg_s ||
        !isfinite(chassis->motion_limits.max_motor_speed_deg_s) ||
        chassis->motion_limits.max_motor_speed_deg_s <= 0.0f ||
        !IsSupportedDJIMotor(&chassis->chassis_motor_config, 0u))
        return 0u;

    if (chassis->wheel_type != MECANUM_WHEEL && chassis->wheel_type != OMNI_WHEEL)
        return 0u;
#if defined(MC02_PROFILE_ROBOT)
    // 启动校验使用与控制周期相同的计算入口，提前发现滚动方向和轮组秩不足。
    const ChassisTwist_t zero_twist = {0.0f, 0.0f, 0.0f};
    ChassisWheelSpeeds_t wheel_speeds;
#if MC02_CHASSIS_IS_OMNI
    if (chassis->wheel_type != OMNI_WHEEL ||
        OmniCalculate(&zero_twist, &chassis->omni_geometry, &wheel_speeds) !=
            CHASSIS_KINEMATICS_OK)
        return 0u;
#else
    const MecanumGeometry_t geometry = {
        .half_base_m = measure->wheel_base * 0.0005f,
        .half_track_m = measure->track_width * 0.0005f,
        .center_x_m = measure->center_gimbal_offset_x * 0.001f,
        .center_y_m = -measure->center_gimbal_offset_y * 0.001f,
        .radius_m = measure->radius_wheel * 0.001f,
    };
    if (chassis->wheel_type != MECANUM_WHEEL ||
        MecanumCalculate(&zero_twist, &geometry, &wheel_speeds) !=
            CHASSIS_KINEMATICS_OK)
        return 0u;
#endif
#endif
    // 检查四个驱动轮都有合法且互不重复的电调编号。
    for (unsigned index = 0u; index < 4u; ++index)
    {
        if (chassis->chassis_motor_id[index] == 0u || chassis->chassis_motor_id[index] > 8u)
            return 0u;
        for (unsigned previous = 0u; previous < index; ++previous)
            if (chassis->chassis_motor_id[previous] == chassis->chassis_motor_id[index])
                return 0u;
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

#if defined(MC02_PROFILE_ROBOT) && MC02_FEATURE_CHASSIS
    if (candidate->chassis_param.motion_limits.configured == 0u)
        return ROBOT_CONFIG_BAD_PARAMETERS;
    if ((MC02_CHASSIS_IS_OMNI != 0 && candidate->chassis_param.wheel_type != OMNI_WHEEL) ||
        (MC02_CHASSIS_IS_OMNI == 0 && candidate->chassis_param.wheel_type != MECANUM_WHEEL))
        return ROBOT_CONFIG_BAD_PARAMETERS;
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
