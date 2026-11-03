/** @file HT04.h
* @brief HT04 独立任务控制接口，公开各电机 PID、反馈和前馈快照来源。
* 接收速度使用每电机独立的五点平均；闭环输入失败时当前周期发送零力矩。
 */
#ifndef HT04_H
#define HT04_H

#include <stdint.h>
#include "../../../bsp/can/platform_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "ht_motor_task.h"

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
    Motor_Output_Axis_Config_s output_axis;

    Motor_Control_Setting_s motor_settings;
    MotorPIDClock_t pid_clock; // 独立记录控制调用时刻，不复用反馈计时。

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    Motor_Signal_Source_s other_angle_feedback;
    Motor_Signal_Source_s other_speed_feedback;
    Motor_Signal_Source_s speed_feedforward;
    Motor_Signal_Source_s current_feedforward;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    PlatformCanRxPort_t can_rx; // 长期保存的逻辑接收端点。
    PlatformCanEndpoint_t can_endpoint; // 保存逻辑总线与收发 ID。
    uint8_t tx_buff[8]; // 复用原有控制帧内容，提交时由平台复制。

    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;
    uint32_t feedback_timestamp_ms;
    uint32_t feedback_sequence;
} HTMotorInstance;

/* HT电机模式,初始化时自动进入CMD_MOTOR_MODE*/
typedef enum
{
    CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    CMD_RESET_MODE = 0xfd,   // 停止
    CMD_ZERO_POSITION = 0xfe // 将当前的位置设置为编码器零位
} HTMotor_Mode_t;

/** @brief 注册 HT04 电机并准备所选 PID 与外部快照来源。
* @param[in] config 只读配置；输出轴配置须有效，实际参与控制的 PID 与外部来源须完整。
* @return 长期保存的对象；空参数、无效输出轴、来源元数据或 PID 配置返回 NULL；
*        成功后再调用 HTMotorControlInit() 创建任务。
* @note 输出轴闭环参考、反馈和 PID 输入使用 rad、rad/s；当前环保留驱动设备力矩数值。
*       外部来源上下文须在任务运行期间有效。未参与分支的来源和 PID 配置可以为空或未初始化。
*       初始化当前仍会发送设备模式命令；CAN／Daemon 回退和实例容量继续由 MC02-0051 跟踪。
 */
HTMotorInstance *HTMotorInit(const Motor_Init_Config_s *config);

/** @brief 复制 HT04 的统一单位反馈快照；未支持的电压字段保持无效。 */
Motor_Feedback_Status_e HTMotorGetFeedbackSnapshot(const HTMotorInstance *motor,
                                                    uint32_t now_ms,
                                                    Motor_Feedback_Snapshot_s *snapshot);

/** @brief 保存目标供独立任务计算。
* @param[in,out] motor 有效对象，不能为 NULL。
* @param[in] ref 角度环目标用输出轴rad，速度环目标用输出轴rad/s；直接输出按T_MIN～T_MAX协议力矩数值处理。
* @note 数值须有限；否则控制任务本周期发送零力矩并清除 PID 历史。real_current 由协议力矩区间还原，不能按变量名直接当作 A。
 */
void HTMotorSetRef(HTMotorInstance *motor, float ref);

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
* @note 初始化时未参与计算的 PID 可保持未初始化；切换到该环后若 PID 或所需数据无效，本周期写零力矩并清除环历史。
 */
void HTMotorOuterLoop(HTMotorInstance *motor, Closeloop_Type_e type);

/** @brief 先发零目标报文，再发将当前位置置零的命令，两次各等待 5 ms。
* @param[in] motor 有效 CAN 电机对象，不检查 NULL 或发送结果，须确认实际设备允许置零。
 */
void HTMotorCalibEncoder(HTMotorInstance *motor);

#endif // !HT04_H#define HT04_H
