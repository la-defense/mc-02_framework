/** @file config.h
* @brief 机器人机械、电机和控制参数类型，以及只读配置读取入口。
* 当前只有哨兵配置；未确认的实物参数仍保持未确认状态，不用测试数值替代。
 */
#pragma once

#include "build_selection.h"
#include "chassis_motion.h"
#include "controller.h"
#include "motor_def.h"
#include "robot_alarm.h"
#include "topics/robot_can_bridge_config.h"
#include "referee_ui_config.h"
#include "shoot_config_types.h"

/*
 * 机器人配置框架. VISION_USE_VCP / VISION_USE_UART / ONE_BOARD 等板级宏
 * 仍保留在 robot_def.h 中定义, 此处不重复, 避免冲突.
 */

/* 当前机器人类型(决定 RobotConfigInit 返回哪套配置) */
#define ROBOT_CONFIG_SUPPORTED_ID SENTINEL_ROBOT

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
    STEER_WHEEL,
    WHEEL_LEG,
    HALF_STEER_WHEEL
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
    ChassisMotionLimits_s motion_limits;
    OmniGeometry_t omni_geometry;

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
    // 小 yaw、底盘对齐偏置及 pitch 上下限，角度单位均为度。
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
    float friction_default_speed_rad_s; // 摩擦轮输出轴默认目标速度，rad/s
    float loader_reverse_speed_rad_s; // 拨弹盘输出轴反转目标速度，rad/s
} Shoot_Config_s;

// 机器人参数结构体
typedef struct
{
    Chassis_Config_s chassis_param;
    Gimbal_Config_s gimbal_param;
    Shoot_Config_s shoot_param;
    PlatformImuPort_t imu_sensor; // 姿态解算使用的逻辑 IMU，由板级适配绑定。
    PlatformPwmChannel_t imu_heater_pwm; // 温控输出使用的逻辑 PWM。
    PlatformAdcChannel_t imu_heater_supply; // 温控检查电源电压使用的逻辑 ADC。
    PlatformUartPort_t referee_uart; // 裁判接收使用的逻辑串口，由板级适配绑定。
    PlatformUartPort_t vision_uart; // 上位机 UART 模式使用的逻辑串口；USB 模式忽略。
    PlatformUartPort_t remote_uart; // 遥控接收使用的逻辑串口，由板级适配绑定。
    const RobotAlarmConfig_t *alarm_config; // 长期只读的状态与故障提示表。
    const RefereeUIConfig_t *referee_ui_config; // 可选的只读裁判界面布局；不影响裁判接收和控制许可。
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)
    RobotCanBridgeConfig_t can_bridge; // 本板的板间总线、ID和底盘话题。
#endif
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
typedef enum {
    ROBOT_CONFIG_OK = 0, ROBOT_CONFIG_ARGUMENT, ROBOT_CONFIG_UNSUPPORTED,
    ROBOT_CONFIG_BAD_PARAMETERS
} RobotConfigStatus_t;
/** @brief 选择唯一已实现车型；失败清除活动选择，不退回其他车型。
 * @param[in] id 车型编号，当前仅支持SENTINEL_ROBOT。
 * @param[out] output 长期只读配置，不可为NULL；失败时保持输出对象原值。
 * @return OK、ARGUMENT、UNSUPPORTED或BAD_PARAMETERS；调用不操作硬件。
 */
RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output);
/** @brief 选择车型并返回其只读配置，兼容历史调用接口。
 * @param[in] id 机器人类型；当前仅支持 SENTINEL_ROBOT。
 * @return 成功时返回长期有效的内部配置地址；选择失败时返回NULL并清除当前选择。
 */
const Robot_Config_s *RobotConfigInit(uint8_t id);
/** @brief 读取已选车型的完整只读配置。
 * @return 配置长期有效；尚未成功选择车型时返回NULL。
 */
const Robot_Config_s *RobotConfigFeed(void);
/** @brief 取得已选配置中的底盘部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Chassis_Config_s *ChassisConfigFeed(void);
/** @brief 取得已选配置中的云台部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Gimbal_Config_s *GimbalConfigFeed(void);
/** @brief 取得已选配置中的发射部分。
* @return 内部字段地址，长期有效；未初始化或未启用此模块时返回NULL。
 */
const Shoot_Config_s *ShootConfigFeed(void);
/** @brief 取得已选配置中的裁判 UI 布局。
* @return 长期只读布局；未完成 RobotConfigInit() 或车型未配置时返回NULL。
*/
const RefereeUIConfig_t *RefereeUIConfigFeed(void);
