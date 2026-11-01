/** @file config.h
 * @brief 云台主机测试使用的最小车型配置结构，不连接板级配置和真实电机。
 */
#ifndef MC02_GIMBAL_MODE_TEST_CONFIG_H
#define MC02_GIMBAL_MODE_TEST_CONFIG_H

#include "motor_def.h"

typedef enum
{
    SINGLE_GIMBAL = 0,
    MINI_GIMBAL,
    DUAL_GIMBAL,
} Gimbal_Type_e;

typedef struct
{
    float mini_yaw_offset;
    float yaw_offset;
    float pitch_max_angle;
    float pitch_min_angle;
} Gimbal_Measure_s;

typedef struct
{
    Gimbal_Type_e gimbal_type;
    Gimbal_Measure_s gimbal_offset;
    Motor_Init_Config_s yaw_motor_config;
    Motor_Init_Config_s mini_yaw_motor_config;
    Motor_Init_Config_s pitch_motor_config;
} Gimbal_Config_s;

const Gimbal_Config_s *GimbalConfigFeed(void);

#endif
