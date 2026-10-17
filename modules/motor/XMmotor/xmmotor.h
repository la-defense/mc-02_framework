/** @file xmmotor.h
 * @brief XM 电机保留实现，默认只分配软件对象，不注册 CAN、Daemon 或创建控制任务。
 * 硬件路径由 XM_MOTOR_ENABLE 包围，启用前还须适配本地扩展帧接口，不能直接按已完成驱动使用。
 */
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

/** @brief 保存控制设置和 PID；默认没有硬件收发。
 * @param[in] config 有效配置，不检查 NULL 或内存失败；反馈指针须长期有效，实例最多四个。
 * @return 模块保存的对象；不检查容量，不能用返回对象判断 CAN 已接入。
 */
XMMotorInstance *XMMotorInit(Motor_Init_Config_s *config);

/** @brief 仅保存参考值；默认配置没有控制任务执行此值。
 * @param[in,out] motor 有效对象，不能为 NULL。
 * @param[in] ref 原始控制目标，单位随所选反馈及控制环决定，本入口不换算或限制范围。
 */
void XMMotorSetRef(XMMotorInstance *motor, float ref);
/** @brief 改写反馈来源。
 * @param[in,out] motor 有效对象，不能为 NULL。
 * @param[in] loop ANGLE_LOOP 或 SPEED_LOOP，其他值打印错误。
 * @param[in] type 电机反馈或外部有效指针，不在此处检查指针。
 */
void XMMotorChangeFeed(XMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
/** @brief 设置软件运行标记，不在此处发送使能命令。
 * @param[in,out] motor 有效对象，不能为 NULL。
 */
void XMMotorEnable(XMMotorInstance *motor);
/** @brief 设置软件停止标记；仅在硬件任务启用时用于覆盖输出。
 * @param[in,out] motor 有效对象，不能为 NULL。
 */
void XMMotorStop(XMMotorInstance *motor);
/** @brief 预留置零入口，目前为空，不发送任何命令。
 * @param[in] motor 当前未使用。
 */
void XMMotorCaliEncoder(XMMotorInstance *motor);
/** @brief 默认只记录已注册数量；硬件路径启用后才逐对象创建任务，重复调用未受保护。 */
void XMMotorControlInit();
#endif // !XMMOTOR_H
