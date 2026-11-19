#ifndef MC02_HOST_CHASSIS_CONFIG_H
#define MC02_HOST_CHASSIS_CONFIG_H

#include <stdint.h>
#include "chassis_motion.h"
#include "dji_motor.h"

typedef enum {
    MECANUM_WHEEL = 0,
    OMNI_WHEEL,
    STEER_WHEEL,
    WHEEL_LEG,
    HALF_STEER_WHEEL
} Wheel_Type_e;

typedef enum {
    MOTOR_LF = 0,
    MOTOR_RF,
    MOTOR_LB,
    MOTOR_RB
} Motor_Turn_e;

typedef struct {
    Wheel_Type_e wheel_type;
    MecanumGeometry_t mecanum_geometry; // 与生产配置同为 SI 几何。
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;
    uint32_t odometry_max_gap_ms; // 与生产配置同义，0 关闭，正值限制积分间隔。
    Motor_Init_Config_s chassis_motor_config;
    uint8_t chassis_motor_id[4];
} Chassis_Config_s;

const Chassis_Config_s *ChassisConfigFeed(void);

#endif
