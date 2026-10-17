/** @file LK9025.h
* @brief 保留的 LK 多电机控制，一条总线共用首个实例发送；启用前须核对当前旧控制路径。
 */
#ifndef LK9025_H
#define LK9025_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define LK_MOTOR_MX_CNT 4 // 最多允许4个LK电机使用多电机指令,挂载在一条总线上

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef struct // 9025
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
    float *current_feedforward_ptr; // 电流前馈指针
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;

} LKMotorInstance;

/** @brief 分配 LK 对象，改写 CAN 编号与回调并注册离线计数，初始为运行标记。
* @param[in,out] config 有效配置；同组编号应从 1 连续排列，最多四个且位于同一总线。
* @return 长期保存对象，当前没有检查空参数、容量、重复分配或底层注册失败，不提供失败保证。
 */
LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config);

/** @brief 保存参考值供下一轮控制使用，不自动换单位。
* @param[in,out] motor 有效对象，不能为 NULL。
* @param[in] ref 须与所选反馈和 PID 配置使用相同数值单位；电机速度反馈为 rad/s，单圈及总角度为 deg。
* @note 当前角度分支默认取 real_current，速度分支也调用 angle_PID，须按实际实现核对后再选用闭环。
 */
void LKMotorSetRef(LKMotorInstance *motor, float ref);

/** @brief 遍历已注册对象、填写每电机两字节原始输出，再用首个实例发送一帧；无注册对象时不发送。 */
void LKMotorControl();

/** @brief 设置停止标记，下一次控制将对应两字节输出清零；保留目标值。
* @param[in,out] motor 有效实例，不能为 NULL。
 */
void LKMotorStop(LKMotorInstance *motor);

/** @brief 设置运行标记，下一次控制继续使用已保存目标。
* @param[in,out] motor 有效实例，不能为 NULL。
 */
void LKMotorEnable(LKMotorInstance *motor);

/** @brief 读取对应 Daemon 是否仍有剩余计数。
* @param[in] motor 有效实例，不能为 NULL。
* @return 1 为计数未耗尽，0 为离线；不代表已允许输出。
 */
uint8_t LKMotorIsOnline(LKMotorInstance *motor);

#endif // LK9025_H
