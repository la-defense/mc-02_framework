/** @file config.h
 * @brief 仅用于编译检查 robot_def.c 的双板角色功能宏组合。
 */
#ifndef MC02_ROBOT_DEF_ROLE_FAKE_CONFIG_H
#define MC02_ROBOT_DEF_ROLE_FAKE_CONFIG_H
#include <stdint.h>
#include "mecanum.h"
#include "omni.h"
#include "../../../application/config/shoot_config_types.h"
#include "../../../modules/motor/motor_def.h" // 使用与生产相同的电机、控制环和 PID 配置。

typedef enum { MECANUM_WHEEL = 0, OMNI_WHEEL = 1 } Wheel_Type_e;
typedef struct {
    uint8_t device_id; // 车型校验读取每轮电调编号。
    Motor_Axis_Direction_e direction; // 车型校验读取每轮安装方向。
} ChassisWheelMotorConfig_t;
typedef struct {
    uint8_t configured;
    float max_wz_deg_s, rotate_wz_deg_s, max_motor_speed_deg_s;
} ChassisMotionLimits_s;
typedef struct {
    Wheel_Type_e wheel_type;
    MecanumGeometry_t mecanum_geometry; // 使用和生产配置相同的 SI 类型。
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;
    uint32_t odometry_max_gap_ms; // 用于检查角色配置的可选里程计间隔，单位 ms。
    Motor_Init_Config_s chassis_motor_config;
    ChassisWheelMotorConfig_t wheel_motors[CHASSIS_WHEEL_COUNT];
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
    struct { PlatformCanBus_t can_bus; } can_bridge; // 双板 CAN 需求测试使用的逻辑总线。
    const RobotAlarmConfig_t *alarm_config;
    const RefereeUIConfig_t *referee_ui_config;
} Robot_Config_s;
typedef enum {
    ROBOT_CONFIG_OK = 0, ROBOT_CONFIG_ARGUMENT, ROBOT_CONFIG_UNSUPPORTED,
    ROBOT_CONFIG_BAD_PARAMETERS
} RobotConfigStatus_t;
enum { SENTINEL_ROBOT = 6 };

#endif
