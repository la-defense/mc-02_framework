/** @file config.h
 * @brief 仅用于编译检查 robot_def.c 的双板角色功能宏组合。
 */
#ifndef MC02_ROBOT_DEF_ROLE_FAKE_CONFIG_H
#define MC02_ROBOT_DEF_ROLE_FAKE_CONFIG_H
#include <stdint.h>

typedef enum { MOTOR_AXIS_DIRECTION_REVERSE = -1, MOTOR_AXIS_DIRECTION_NORMAL = 1 }
    Motor_Axis_Direction_e;
typedef struct {
    struct { void *can_handle; uint32_t tx_id; } can_init_config;
    struct { float gear_ratio; Motor_Axis_Direction_e direction; } output_axis;
    int motor_type;
} Motor_Init_Config_s;
enum { GM6020 = 1, M3508 = 2, M2006 = 3 };
enum { MOTOR_DIRECTION_NORMAL = 0, MOTOR_DIRECTION_REVERSE = 1 };

typedef enum { MECANUM_WHEEL = 0, OMNI_WHEEL = 1 } Wheel_Type_e;
typedef struct {
    float x_m, y_m, drive_x, drive_y, radius_m;
} ChassisWheelGeometry_t;
typedef struct { ChassisWheelGeometry_t wheels[4]; } OmniGeometry_t;
typedef struct {
    float wheel_base, track_width, radius_wheel, reduction_ratio_wheel;
    float center_gimbal_offset_x, center_gimbal_offset_y;
} Wheel_Measure_s;
typedef struct {
    uint8_t configured;
    float max_wz_deg_s, rotate_wz_deg_s, max_motor_speed_deg_s;
} ChassisMotionLimits_s;
typedef struct {
    Wheel_Type_e wheel_type;
    Wheel_Measure_s wheel_measure;
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;
    Motor_Init_Config_s chassis_motor_config;
    uint8_t chassis_motor_id[8];
} Chassis_Config_s;

typedef enum { SINGLE_GIMBAL = 0 } Gimbal_Type_e;
typedef struct {
    float yaw_offset, pitch_min_angle, pitch_max_angle;
} Gimbal_Measure_s;
typedef struct {
    Gimbal_Type_e gimbal_type;
    Gimbal_Measure_s gimbal_offset;
    Motor_Init_Config_s yaw_motor_config, pitch_motor_config;
} Gimbal_Config_s;

typedef enum { DUAL_SHOOT = 0, THREE_SHOOT, FOUR_SHOOT } Shoot_Type_e;
typedef struct {
    Shoot_Type_e shoot_type;
    Motor_Init_Config_s friction_motor_config, loader_motor_config;
    uint8_t friction_motor_id[4];
    int friction_motor_reverse_flag[4];
    uint8_t loader_slot_num;
    float loader_gearbox_ratio, loader_disk_ratio;
    float friction_default_speed_rad_s, loader_reverse_speed_rad_s;
} Shoot_Config_s;
typedef struct RobotAlarmConfig RobotAlarmConfig_t;
typedef struct RefereeUIConfig RefereeUIConfig_t;
typedef struct {
    Chassis_Config_s chassis_param;
    Gimbal_Config_s gimbal_param;
    Shoot_Config_s shoot_param;
    const RobotAlarmConfig_t *alarm_config;
    const RefereeUIConfig_t *referee_ui_config;
} Robot_Config_s;
typedef enum {
    ROBOT_CONFIG_OK = 0, ROBOT_CONFIG_ARGUMENT, ROBOT_CONFIG_UNSUPPORTED,
    ROBOT_CONFIG_BAD_PARAMETERS
} RobotConfigStatus_t;
enum { SENTINEL_ROBOT = 6 };
#define MC02_CHASSIS_IS_OMNI 1

#endif
