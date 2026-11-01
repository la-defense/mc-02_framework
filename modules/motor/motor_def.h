/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "controller.h"
#include "motor_pid.h"
#include "../../bsp/board/platform_resources.h"
#include "motor_feedback.h"
#include "stdint.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum
{
    OPEN_LOOP = 0b0000,
    CURRENT_LOOP = 0b0001,
    SPEED_LOOP = 0b0010,
    ANGLE_LOOP = 0b0100,

    // only for checking
    SPEED_AND_CURRENT_LOOP = 0b0011,
    ANGLE_AND_SPEED_LOOP = 0b0110,
    ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum
{
    FEEDFORWARD_NONE = 0b00,
    CURRENT_FEEDFORWARD = 0b01,
    SPEED_FEEDFORWARD = 0b10,
    CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 外部反馈来源由只读回调复制完整快照，不向控制器暴露可写数据地址。 */
typedef enum
{
    MOTOR_FEED = 0,
    OTHER_FEED,
} Feedback_Source_e;

/*
 * @brief 旧驱动方向兼容字段。
 * DJI、LK9025 和 HT04 要求保持 NORMAL；方向统一配置在 output_axis.direction。
 * DM 的直接力矩协议仍使用 motor_reverse_flag。
 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,
    MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/*
 * @brief 旧反馈输出方向兼容字段。
 * DJI、LK9025 和 HT04 要求保持 NORMAL；新配置使用 output_axis.direction。
 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,
    FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

/* 电机工作状态 */
/* 电机离线标志 */
typedef enum
{
    MOTOR_ONLINE = 0,
    MOTOR_OFFLINE = 1,
} Motor_Online_Flag_e;

/* 电机控制设置包含闭环和来源；三类已迁移驱动的方向只由输出轴配置管理。 */
typedef struct
{
    Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
    Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
    Motor_Reverse_Flag_e motor_reverse_flag;       // DJI／LK／HT须为NORMAL；DM直接力矩仍读取此字段。
    Feedback_Reverse_Flag_e feedback_reverse_flag; // DJI／LK／HT须为NORMAL；方向使用output_axis.direction。
    Feedback_Source_e angle_feedback_source;       // 角度反馈类型
    Feedback_Source_e speed_feedback_source;       // 速度反馈类型
    Feedfoward_Type_e feedforward_flag;            // 前馈标志
    uint8_t enable; // 是否启用

} Motor_Control_Setting_s;

/* 电机控制器保存外部快照来源、三环PID和本实例参考值。 */
typedef struct
{
    Motor_Signal_Source_s other_angle_feedback; // 角度来源；输出轴环使用 rad 与显式参考系。
    Motor_Signal_Source_s other_speed_feedback; // 速度来源；输出轴环使用 rad/s 与显式参考系。
    Motor_Signal_Source_s speed_feedforward; // 速度前馈，目标及反馈同为输出轴 rad/s。
    Motor_Signal_Source_s current_feedforward; // 当前／力矩前馈，单位由驱动能力声明。

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float pid_ref; // 角度／速度环使用输出轴rad或rad/s；电流环使用设备电流量纲。
} Motor_Controller_s;

/**
 * @brief 电机控制器初始化结构体，包含三环 PID 参数及可选的外部快照来源。
 * @note DJI、LK9025、HT04 的角度环使用 rad、速度环使用 rad/s；PID measure 与 ref 必须同单位，
 *       增益、死区、积分限幅及输出限幅也按各 PID 输入／输出单位解释。仅实际参与的环须提供有效 PID。
 *       选择 OTHER_FEED 或启用前馈时，来源单位和参考系须匹配；来源上下文须长于电机实例生命周期。
 */
typedef struct
{
    // 来源回调复制带单位、参考系、采样时间和有效标记的值；上下文由调用方长期保存。
    Motor_Signal_Source_s other_angle_feedback;
    Motor_Signal_Source_s other_speed_feedback;
    Motor_Signal_Source_s speed_feedforward;
    Motor_Signal_Source_s current_feedforward;

    PID_Init_Config_s current_PID;
    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;
} Motor_Controller_Init_s;

/** @brief 电机驱动初始化配置；传给驱动的 const 接口，初始化成功后实例会复制所需字段。 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    Motor_Output_Axis_Config_s output_axis; // 减速比为转子／输出轴，角度／速度闭环映射到输出轴坐标。
    Motor_Type_e motor_type;
    PlatformCanEndpoint_t can_init_config; // 逻辑总线及协议 ID，硬件句柄由板级适配绑定。
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
