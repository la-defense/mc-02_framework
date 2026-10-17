/** @file config.h
* @brief 机器人机械、电机和控制参数类型，以及配置读取入口。
* 当前只有哨兵配置；这些结构不表示全部参数都已由实车确认，应用仍有自身使用的常量。
 */
#pragma once

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"

/*
 * 机器人配置框架. VISION_USE_VCP / VISION_USE_UART / ONE_BOARD 等板级宏
 * 仍保留在 robot_def.h 中定义, 此处不重复, 避免冲突.
 */

/* 当前机器人类型(决定 RobotConfigInit 返回哪套配置) */
#define ROBOT_ID SENTINEL_ROBOT

#define RC_TO_YAW_ANGLE 0.002f    // 每个命令周期、每单位摇杆读数增加的 yaw 角度，单位 deg。
#define RC_TO_PITCH_ANGLE 0.001f  // 每个命令周期、每单位摇杆读数增加的 pitch 角度，单位 deg。

// #define _IS_IMU_ROLL    // 是否使用IMU的roll角作为pitch角(由于不常修改,作为宏). 哨兵pitch反馈源依机械安装而定

// 电机位置
typedef enum
{
    MOTOR_LF = 0,
    MOTOR_RF,
    MOTOR_LB,
    MOTOR_RB,
    MOTOR_UP_LF,
    MOTOR_UP_RF,
    MOTOR_UP_LB,
    MOTOR_UP_RB
} Motor_Turn_e;

// 机器人底盘类型
typedef enum
{
    MECANUM_WHEEL = 0,
    OMNI_WHEEL,
    STEER_WHEEL
} Wheel_Type_e;

// 底盘轮组参数
typedef struct
{
    // 云台中心偏移、轴距、轮距和轮半径均用 mm；减速比无单位。
    float center_gimbal_offset_x;
    float center_gimbal_offset_y;

    float wheel_base;
    float track_width;
    float radius_wheel;

    float reduction_ratio_wheel;
} Wheel_Measure_s;

typedef struct
{
    Wheel_Type_e wheel_type;
    Wheel_Measure_s wheel_measure;

    Motor_Init_Config_s chassis_motor_config;
    PID_Init_Config_s chassis_follow_gimbal_PID;

    uint8_t chassis_motor_id[8];
} Chassis_Config_s;

// 机器人云台类型
typedef enum
{
    SINGLE_GIMBAL = 0,
    MINI_GIMBAL,
    DUAL_GIMBAL,
} Gimbal_Type_e;

// 云台归中参数
typedef struct
{
    // 云台零位偏移和俯仰上下限，单位为度，具体反馈来源仍由应用选择。
    float mini_yaw_offset;
    float yaw_offset;
    float pitch_max_angle, pitch_min_angle;
} Gimbal_Measure_s;

typedef struct
{
    Gimbal_Type_e gimbal_type;
    Gimbal_Measure_s gimbal_offset;
    Motor_Init_Config_s yaw_motor_config;
    Motor_Init_Config_s mini_yaw_motor_config;
    Motor_Init_Config_s pitch_motor_config;
} Gimbal_Config_s;

// 机器人发射类型
typedef enum
{
    DUAL_SHOOT = 0,
    THREE_SHOOT,
    FOUR_SHOOT,
} Shoot_Type_e;

typedef struct
{
    Shoot_Type_e shoot_type;
    Motor_Init_Config_s friction_motor_config;
    Motor_Init_Config_s loader_motor_config;

    uint8_t friction_motor_id[4];
    Motor_Reverse_Flag_e friction_motor_reverse_flag[4];
    uint8_t loader_slot_num;     // 拨弹盘槽位数(每转一圈发射的弹丸数)
    float loader_gearbox_ratio;  // 电机转子 -> 输出轴减速比(M2006=36, M3508=19)
    float loader_disk_ratio;     // 输出轴 -> 拨弹盘减速比(哨兵=1)
} Shoot_Config_s;

// 机器人参数结构体
typedef struct
{
    Chassis_Config_s chassis_param;
    Gimbal_Config_s gimbal_param;
    Shoot_Config_s shoot_param;
} Robot_Config_s;

// 机器人类型
typedef enum
{
    HERO_ROBOT = 1,
    ENGINEER_ROBOT,
    INFANTRY_ROBOT_3,
    INFANTRY_ROBOT_4,
    INFANTRY_ROBOT_5,
    AERIAL_ROBOT,
    SENTINEL_ROBOT,
    DART_ROBOT,
} Robot_Type_e;

#include "robot_def.h"

/* 配置入口(在 robot_def.c 中实现) */
/** @brief 选择并保存供后续 Feed 使用的机器人配置。
* @param[in] id 机器人类型，当前所有未实现或未知编号都会返回哨兵配置。
* @return 长期有效的内部配置地址，不可释放；再次调用不会恢复已改写的静态配置。
 */
Robot_Config_s *RobotConfigInit(uint8_t id);
/** @brief 取得已选配置中的底盘部分。
* @return 内部字段地址，长期有效；必须先成功调用 RobotConfigInit()，此处不检查是否已初始化。
 */
Chassis_Config_s *ChassisConfigFeed(void);
/** @brief 取得已选配置中的云台部分。
* @return 内部字段地址，长期有效；必须先成功调用 RobotConfigInit()，此处不检查是否已初始化。
 */
Gimbal_Config_s *GimbalConfigFeed(void);
/** @brief 取得已选配置中的发射部分。
* @return 内部字段地址，长期有效；必须先成功调用 RobotConfigInit()，此处不检查是否已初始化。
 */
Shoot_Config_s *ShootConfigFeed(void);
