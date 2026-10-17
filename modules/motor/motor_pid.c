/** @file motor_pid.c
 * @brief 在电机调用侧检查计算间隔，统一处理未启用环和历史清除。
 */
#include "motor_pid.h"
#include <stddef.h>

PID_Status_e MotorPIDInit(PIDInstance *pid, const PID_Init_Config_s *config, uint8_t enabled)
{
    if (pid == NULL)
        return PID_BAD_ARGUMENT;
    if (!enabled)
    {
        // 未启用环允许省略参数，不用全零配置冒充已经初始化。
        *pid = (PIDInstance){0};
        return PID_OK;
    }
    return PIDInit(pid, config);
}

PID_Status_e MotorPIDBegin(MotorPIDClock_t *clock, uint64_t now_us, float *dt_s)
{
    if (clock == NULL || dt_s == NULL)
        return PID_BAD_ARGUMENT;
    uint64_t previous = clock->previous_us;
    uint8_t started = clock->started;
    clock->previous_us = now_us;
    clock->started = 1u;
    // 每次记录当前时刻，异常后下一轮不会继续累计很久以前的间隔。
    if (!started || now_us < previous || now_us - previous < 10u || now_us - previous > 50000u)
    {
        clock->last_status = PID_BAD_DT;
        return PID_BAD_DT;
    }
    // 用除法换算，使10μs和50ms换算后与配置中的边界值一致。
    *dt_s = (float)(now_us - previous) / 1000000.0f;
    clock->last_status = PID_OK;
    return PID_OK;
}

void MotorPIDResetLoops(PIDInstance *angle, PIDInstance *speed, PIDInstance *current)
{
    (void)PIDReset(angle);
    (void)PIDReset(speed);
    (void)PIDReset(current);
}
