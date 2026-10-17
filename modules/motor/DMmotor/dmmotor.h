/** @file dmmotor.h
* @brief 达妙旧力矩控制接口；注册及独立任务均未接入当前统一初始化失败检查。
*/
#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

/** @brief 分配对象、改写 CAN 配置回调并注册电机，发送使能和当前位置置零命令。
* @param[in,out] config 有效配置，初始化期间会改写 can_init_config；不检查 NULL、容量或注册分配失败。
* @return 长期保存的对象，无释放入口；最多四个，先注册全部电机，再调用 DMMotorControlInit() 创建任务。
* @note 调用含共约 0.3 s 的 DWT 等待及实际 CAN 命令，须先准备外设和计时。
*/
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/** @brief 保存当前力矩参考值，不直接发送。
* @param[in,out] motor 有效实例，不能为 NULL。
* @param[in] ref 须为正常且在 DM_T_MIN～DM_T_MAX 内的协议力矩数值；接口不检查。
* @note 任务当前编码的是原始 pid_ref，前面临时 set 的反向及范围处理未用于该字段，不能依赖自动限制。
*/
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/** @brief 只改写最外层控制设置；当前任务仅发送力矩，没有使用这些 PID 环。
* @param[in,out] motor 有效对象，不能为 NULL。
* @param[in] closeloop_type 保存的控制环类型。
*/
void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

/** @brief 设置运行标记，任务随后发送保存的力矩参考。
* @param[in,out] motor 有效实例，不能为 NULL。
*/
void DMMotorEnable(DMMotorInstance *motor);

/** @brief 设置停止标记；任务继续发送零力矩以保留反馈，不发送退出电机模式命令。
* @param[in,out] motor 有效实例，不能为 NULL。
*/
void DMMotorStop(DMMotorInstance *motor);
/** @brief 发送将当前位置置零的命令，并用 DWT 等待 0.1 s。
* @param[in] motor 已注册 CAN 的有效实例，不能为 NULL；不报告发送结果。
*/
void DMMotorCaliEncoder(DMMotorInstance *motor);
/** @brief 为每个已注册电机创建约每 2 ms 运行的任务；无电机时返回，未检查创建失败。
* @note 注册结束后仅调用一次，重复调用会再创建任务。
*/
void DMMotorControlInit();
#endif // !DMMOTOR