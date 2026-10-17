/** @file motor_pid.h
 * @brief 电机调用侧的 PID 初始化和周期计时，不读取硬件时钟。
 * 驱动传入现有微秒时间，每电机保存独立时刻，同一轮串级环共享结果。
 */
#ifndef MC02_MOTOR_PID_H
#define MC02_MOTOR_PID_H
#include "../algorithm/controller.h"

#define MOTOR_PID_DT_MIN_S 0.00001f
#define MOTOR_PID_DT_MAX_S 0.05f

/** @brief 控制调用时间，与 CAN 反馈采样时间分别保存。 */
typedef struct
{
    uint64_t previous_us;
    uint8_t started;
    PID_Status_e last_status;
} MotorPIDClock_t;

/**
 * @brief 初始化启用环；未启用环清零并保持未初始化。
 * @param[out] pid 调用者保存的实例，不能为 NULL。
 * @param[in] config 启用时须提供有效参数，包含显式 dt 范围。
 * @param[in] enabled 非0表示初始化该环。
 * @return PID_OK 表示成功或该环未启用，失败时返回 PID 错误。
 */
PID_Status_e MotorPIDInit(PIDInstance *pid, const PID_Init_Config_s *config, uint8_t enabled);

/**
 * @brief 计算本轮实际间隔；首次调用或间隔异常时更新基准并拒绝本轮。
 * @param[in,out] clock 每电机独立、首次使用前清零的计时对象。
 * @param[in] now_us 现有时基读取的单调微秒数。
 * @param[out] dt_s 成功时交付10μs～50ms的秒数，失败时保持原值。
 * @return PID_OK 表示可以计算；首次、时间回退或间隔超界返回 PID_BAD_DT。
 */
PID_Status_e MotorPIDBegin(MotorPIDClock_t *clock, uint64_t now_us, float *dt_s);

/**
 * @brief 清除三个控制环的历史，供本轮失败后重新开始使用。
 * @param[in,out] angle 角度环，可以为 NULL 或未初始化实例。
 * @param[in,out] speed 速度环，可以为 NULL 或未初始化实例。
 * @param[in,out] current 电流环，可以为 NULL 或未初始化实例。
 */
void MotorPIDResetLoops(PIDInstance *angle, PIDInstance *speed, PIDInstance *current);
#endif
