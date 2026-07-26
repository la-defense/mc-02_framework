// #ifndef DRMOTOR_H
// #define DRMOTOR_H
// #include <stdint.h>
// #include "bsp_can.h"
// #include "controller.h"
// #include "motor_def.h"
// #include "daemon.h"

// #define DR_MOTOR_CNT 4

// #define DR_V_MIN  (-45.0f)
// #define DR_V_MAX  45.0f
// #define DR_T_MIN  (-1.8f)
// #define DR_T_MAX   1.8f

// typedef struct
// {
//     uint8_t id;
//     uint8_t state;
//     float velocity;
//     float last_position;
//     float position;
//     float torque;

//     int32_t total_round;
// }DR_Motor_Measure_s;

// typedef struct
// {
//     uint16_t position_des;
//     uint16_t velocity_des;
//     uint16_t torque_des;
//     uint16_t Kp;
//     uint16_t Kd;
// }DRMotor_Send_s;
// typedef struct
// {
//     DR_Motor_Measure_s measure;
//     Motor_Control_Setting_s motor_settings;
//     PIDInstance current_PID;
//     PIDInstance speed_PID;
//     PIDInstance angle_PID;
//     float *other_angle_feedback_ptr;
//     float *other_speed_feedback_ptr;
//     float *speed_feedforward_ptr;
//     float *current_feedforward_ptr;
//     float pid_ref;
//     Motor_Working_Type_e stop_flag;
//     Motor_Online_Flag_e online_flag; // 在线标志

//     CANInstance *motor_can_instace;
//     DaemonInstance* motor_daemon;
//     uint32_t lost_cnt;
// }DRMotorInstance;

// typedef enum
// {
//     DR_CMD_READ_MOTOR = 22001,   // 使能,获取电机实时数据
//     DR_CMD_NO_READ = 38001,      // 使能,获取电机实时数据
//     DR_CMD_NO_CRASH = 31206,     // 关闭碰撞检测
//     DR_CMD_NO_LIMIT = 31208,     // 取消电机角度限制
//     DR_CMD_TEMP_ERROR = 36101,   // 开启电机过热保护
// }DRMotor_Mode_e;

// DRMotorInstance *DRMotorInit(Motor_Init_Config_s *config);

// // void DMMotorSetRef(DMMotorInstance *motor, float ref);
// // void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);
// void DRMotorEnable(DRMotorInstance *motor);
// void DRMotorStop(DRMotorInstance *motor);
// // void DMMotorCaliEncoder(DMMotorInstance *motor);
// void DRMotorControlInit();
// #endif // !DRMOTOR_H
