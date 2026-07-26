#ifndef XMMOTOR_H
#define XMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define XM_MOTOR_CNT 4

#define XM_P_MIN  (-720.0f)
#define XM_P_MAX  720.0f
#define XM_V_MIN  (-30.0f)
#define XM_V_MAX  30.0f
#define XM_T_MIN  (-12.0f)
#define XM_T_MAX   12.0f
#define XM_I_MAX   10.0f
#define XM_I_MIN   -10.0f

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
    float temperature;
    int32_t total_round;
}XM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}XMMotor_Send_s;
typedef struct
{
    XM_Motor_Measure_s measure;                 // 电机测量值
    Motor_Control_Setting_s motor_settings;     // 电机控制设置
    Motor_Controller_s motor_controller;        // 电机控制器

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}XMMotorInstance;

typedef enum
{
    XM_CMD_MOTOR_MODE = 0x03,       // 使能,会响应指令
    XM_CMD_RESET_MODE = 0xfd,       // 停止
    XM_CMD_ZERO_POSITION = 0xfe,    // 将当前的位置设置为编码器零位
    XM_CMD_CURRENT_MODE = 0x12,     // 电流环模式
}XMMotor_Mode_e;

XMMotorInstance *XMMotorInit(Motor_Init_Config_s *config);

void XMMotorSetRef(XMMotorInstance *motor, float ref);
void XMMotorChangeFeed(XMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void XMMotorEnable(XMMotorInstance *motor);
void XMMotorStop(XMMotorInstance *motor);
void XMMotorCaliEncoder(XMMotorInstance *motor);
void XMMotorControlInit();
#endif // !XMMOTOR_H
