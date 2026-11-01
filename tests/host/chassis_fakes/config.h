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
    float center_gimbal_offset_x;
    float center_gimbal_offset_y;
    float wheel_base;
    float track_width;
    float radius_wheel;
    float reduction_ratio_wheel;
} Wheel_Measure_s;

typedef struct {
    Wheel_Type_e wheel_type;
    Wheel_Measure_s wheel_measure;
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;
    Motor_Init_Config_s chassis_motor_config;
    uint8_t chassis_motor_id[4];
} Chassis_Config_s;

const Chassis_Config_s *ChassisConfigFeed(void);

#endif
