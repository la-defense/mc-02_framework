/** @file HT04.h
 * @brief HT04 独立任务控制接口；注册及独立任务均未接入当前统一初始化失败检查。
 */
#ifndef HT04_H
#define HT04_H

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define HT_MOTOR_CNT 4
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_BUFFER_SIZE 5
#define HT_SPEED_BIAS -0.0109901428f // 电机速度偏差,单位rad/s
#define TORQUE_COEF_HT 3.5  // 扭矩系数,单位N.m/A

#define P_MIN -95.5f // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f // Rad/s
#define V_MAX 45.0f
#define T_MIN -18.0f // N·m
#define T_MAX 18.0f
#define KP_MIN 0.0f // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f // N-m/rad/s
#define KD_MAX 5.0f

typedef struct // HT04
{
    float total_angle; // 角度为多圈角度,范围是-95.5~95.5,单位为rad
    float last_angle;

    float speed_rads;
    float speed_buff[SPEED_BUFFER_SIZE];

    float real_current;

    float feed_dt;
    uint32_t feed_cnt;
} HTMotor_Measure_t;

/* HT电机类型定义*/
typedef struct
{
    HTMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_instace;

    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;
} HTMotorInstance;

/* HT电机模式,初始化时自动进入CMD_MOTOR_MODE*/
typedef enum
{
    CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    CMD_RESET_MODE = 0xfd,   // 停止
    CMD_ZERO_POSITION = 0xfe // 将当前的位置设置为编码器零位
} HTMotor_Mode_t;

/** @brief 注册 HT04 电机及离线计数，初始化中发送使能和编码器置零命令。
 * @param[in,out] config 有效配置，会改写 CAN 回调及 id；不检查 NULL、容量或分配注册失败。
 * @return 长期保存的对象，无释放入口，最多四个；随后调用 HTMotorControlInit() 创建独立控制任务。
 * @note 已包含实际 CAN 命令和 DWT 等待，先准备外设与计时；外部角度、速度反馈须用 rad、rad/s。
 */
HTMotorInstance *HTMotorInit(Motor_Init_Config_s *config);

/** @brief 保存目标供独立任务计算。
 * @param[in,out] motor 有效对象，不能为 NULL。
 * @param[in] ref 角度环目标用 deg，速度环目标用 deg/s；直接输出按 T_MIN～T_MAX 的协议力矩数值处理。
 * @note 数值须正常；real_current 由协议力矩区间还原，不能按变量名直接当作 A。
 */
void HTMotorSetRef(HTMotorInstance *motor, float ref);

/** @brief 为每个已注册 HT 电机创建约每 1 ms 运行的任务，无电机时返回。
 * @note 注册完成后只调用一次；不检查任务创建失败，重复调用会新增任务。
 */
void HTMotorControlInit();

/** @brief 只设置停止标记，任务下一轮发送零力矩，并继续保持通信。
 * @param[in,out] motor 有效实例，不能为 NULL。
 */
void HTMotorStop(HTMotorInstance *motor);

/** @brief 设置运行标记，任务重新使用已保存目标。
 * @param[in,out] motor 有效实例，不能为 NULL。
 */
void HTMotorEnable(HTMotorInstance *motor);

/** @brief 改写最外层控制环，下一轮任务按设置计算。
 * @param[in,out] motor 有效实例，不能为 NULL。
 * @param[in] type 角度、速度或直接输出的控制类型，须与其余环配置匹配。
 */
void HTMotorOuterLoop(HTMotorInstance *motor, Closeloop_Type_e type);

/** @brief 先发零目标报文，再发将当前位置置零的命令，两次各等待 5 ms。
 * @param[in] motor 有效 CAN 电机对象，不检查 NULL 或发送结果，须确认实际设备允许置零。
 */
void HTMotorCalibEncoder(HTMotorInstance *motor);

#endif // !HT04_H#define HT04_H