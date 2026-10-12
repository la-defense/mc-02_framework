#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>

typedef enum
{
    CHASSIS_MOTION_MODE_INVALID = 0,
    CHASSIS_MOTION_MODE_NO_FOLLOW,
    CHASSIS_MOTION_MODE_FOLLOW,
    CHASSIS_MOTION_MODE_ROTATE,
} ChassisMotionMode_e;

typedef struct
{
    float vx_mm_s;
    float vy_mm_s;
    float requested_wz_deg_s; /* Must be finite; mode selects the actual wz. */
    float offset_angle_deg;
    ChassisMotionMode_e mode;
} ChassisMotionInput_s;

typedef struct
{
    float wheel_base_mm;
    float track_width_mm;
    float center_gimbal_offset_x_mm;
    float center_gimbal_offset_y_mm;
    float wheel_radius_mm;
    float reduction_ratio;
} ChassisMotionGeometry_s;

typedef struct
{
    uint8_t configured;
    float max_wz_deg_s;
    float rotate_wz_deg_s;
    float max_motor_speed_deg_s;
} ChassisMotionLimits_s;

typedef struct
{
    uint8_t valid;
    float wz_deg_s;
    /* Order: left front, right front, left back, right back; motor shaft deg/s. */
    float motor_speed_deg_s[4];
} ChassisMotionOutput_s;

/* Pure production calculation. Failure always leaves a zero, invalid output. */
uint8_t ChassisMotionCalculate(const ChassisMotionInput_s *input,
                               const ChassisMotionGeometry_s *geometry,
                               const ChassisMotionLimits_s *limits,
                               ChassisMotionOutput_s *output);

#endif
