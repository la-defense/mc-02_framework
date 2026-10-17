/**
 * @file controller.c
 * @brief 实现带输入检查的位置式 PID，计算成功后才保存结果。
 * 参数及使用顺序见 controller.h；临时数据只属于本次调用。
 * @author Wang Hongxi
 * @author modified by neozng
 * @date 2022-11-01
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include <math.h>
#include <stddef.h>

#define PID_INITIALIZED 0x50494431u
#define PID_IMPLEMENTED_OPTIONS 0xf7u

// 检查参数，使非法时间和非有限数值不能进入积分、微分及滤波计算。
static PID_Status_e PIDCheckConfig(const PID_Init_Config_s *c)
{
    if (((uint32_t)c->Improve & ~PID_IMPLEMENTED_OPTIONS) != 0u)
        return PID_UNSUPPORTED_IMPROVEMENT;
    if (!isfinite(c->Kp) || !isfinite(c->Ki) || !isfinite(c->Kd) ||
        !isfinite(c->MaxOut) || !isfinite(c->DeadBand) || !isfinite(c->IntegralLimit) ||
        !isfinite(c->CoefA) || !isfinite(c->CoefB) || !isfinite(c->Output_LPF_RC) ||
        !isfinite(c->Derivative_LPF_RC) || !isfinite(c->DtMin_s) || !isfinite(c->DtMax_s) ||
        c->MaxOut < 0.0f || c->DeadBand < 0.0f || c->IntegralLimit < 0.0f ||
        c->CoefA < 0.0f || c->CoefB < 0.0f || c->Output_LPF_RC < 0.0f ||
        c->Derivative_LPF_RC < 0.0f || c->DtMin_s <= 0.0f || c->DtMax_s < c->DtMin_s ||
        ((c->Improve & PID_ChangingIntegrationRate) &&
         (c->CoefA <= 0.0f || !isfinite(c->CoefA + c->CoefB))))
        return PID_BAD_CONFIG;
    return PID_OK;
}

// 逐项检查计算数据，不把整数计数或枚举按浮点内存解释。
static uint8_t PIDStateIsFinite(const PID_State_t *s)
{
    return isfinite(s->Measure) && isfinite(s->Last_Measure) &&
           isfinite(s->Err) && isfinite(s->Last_Err) && isfinite(s->Last_ITerm) &&
           isfinite(s->Pout) && isfinite(s->Iout) && isfinite(s->Dout) &&
           isfinite(s->ITerm) && isfinite(s->Output) && isfinite(s->Last_Output) &&
           isfinite(s->Last_Dout) && isfinite(s->Ref) && isfinite(s->dt);
}

PID_Status_e PIDInit(PIDInstance *pid, const PID_Init_Config_s *config)
{
    if (pid == NULL || config == NULL)
        return PID_BAD_ARGUMENT;
    PID_Status_e status = PIDCheckConfig(config);
    if (status != PID_OK)
        return status;
    // 先复制配置，允许用实例内原配置重新初始化。
    PIDInstance next = {0};
    next.config = *config;
    next.initialized = PID_INITIALIZED;
    *pid = next;
    return PID_OK;
}

PID_Status_e PIDReset(PIDInstance *pid)
{
    if (pid == NULL)
        return PID_BAD_ARGUMENT;
    if (pid->initialized != PID_INITIALIZED)
        return PID_NOT_INITIALIZED;
    pid->state = (PID_State_t){0};
    return PID_OK;
}

PID_Status_e PIDGetState(const PIDInstance *pid, PID_State_t *state)
{
    if (pid == NULL || state == NULL)
        return PID_BAD_ARGUMENT;
    if (pid->initialized != PID_INITIALIZED)
        return PID_NOT_INITIALIZED;
    *state = pid->state;
    return PID_OK;
}

// 沿用旧规则：用上一次数据检查堵转，参考值接近0或输出很小时不累计。
static uint8_t PIDCheckBlocked(PID_State_t *s, const PID_Init_Config_s *c)
{
    if (fabsf(s->Output) < c->MaxOut * 0.001f || fabsf(s->Ref) < 0.0001f)
        return 1u;
    float ratio = fabsf(s->Ref - s->Measure) / fabsf(s->Ref);
    if (!isfinite(ratio))
        return 0u;
    if (ratio > 0.95f)
    {
        if (s->ERRORHandler.ERRORCount != UINT64_MAX)
            ++s->ERRORHandler.ERRORCount;
    }
    else
        s->ERRORHandler.ERRORCount = 0u;
    if (s->ERRORHandler.ERRORCount > 500u)
        s->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    return 1u;
}

// 按旧顺序限制积分；两个临时值保存在栈上，不与其他实例共享。
static uint8_t PIDLimitIntegral(PID_State_t *s, const PID_Init_Config_s *c)
{
    float next_integral = s->Iout + s->ITerm;
    float previous_output = s->Pout + s->Iout + s->Dout;
    float accumulating = s->Err * s->Iout;
    if (!isfinite(next_integral) || !isfinite(previous_output) || !isfinite(accumulating))
        return 0u;
    if (fabsf(previous_output) > c->MaxOut && accumulating > 0.0f)
        s->ITerm = 0.0f;
    if (next_integral > c->IntegralLimit)
    {
        s->ITerm = 0.0f;
        s->Iout = c->IntegralLimit;
    }
    if (next_integral < -c->IntegralLimit)
    {
        s->ITerm = 0.0f;
        s->Iout = -c->IntegralLimit;
    }
    return 1u;
}

PID_Status_e PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s, float *output)
{
    if (pid == NULL || output == NULL || !isfinite(measure) || !isfinite(ref))
        return PID_BAD_ARGUMENT;
    if (pid->initialized != PID_INITIALIZED)
        return PID_NOT_INITIALIZED;
    const PID_Init_Config_s *c = &pid->config;
    if (!isfinite(dt_s) || dt_s < c->DtMin_s || dt_s > c->DtMax_s)
        return PID_BAD_DT;

    // 在副本上计算；失败不会留下半次积分或更新后的误差。
    PID_State_t next = pid->state;
    if ((c->Improve & PID_ErrorHandle) && !PIDCheckBlocked(&next, c))
        return PID_NUMERIC_ERROR;
    next.dt = dt_s;
    next.Measure = measure;
    next.Ref = ref;
    next.Err = ref - measure;
    if (!isfinite(next.Err))
        return PID_NUMERIC_ERROR;
    if (fabsf(next.Err) > c->DeadBand)
    {
        next.Pout = c->Kp * next.Err;
        next.ITerm = c->Ki * next.Err * dt_s;
        next.Dout = c->Kd * (next.Err - next.Last_Err) / dt_s;
        if (!PIDStateIsFinite(&next))
            return PID_NUMERIC_ERROR;
        if (c->Improve & PID_Trapezoid_Intergral)
        {
            next.ITerm = c->Ki * ((next.Err + next.Last_Err) / 2.0f) * dt_s;
            if (!isfinite(next.ITerm))
                return PID_NUMERIC_ERROR;
        }
        if (c->Improve & PID_ChangingIntegrationRate)
        {
            float accumulating = next.Err * next.Iout;
            if (!isfinite(accumulating))
                return PID_NUMERIC_ERROR;
            // 同向累计时，误差超过完整积分区间后逐步减小本次积分。
            if (accumulating > 0.0f && fabsf(next.Err) > c->CoefB)
            {
                if (fabsf(next.Err) <= c->CoefA + c->CoefB)
                    next.ITerm *= (c->CoefA - fabsf(next.Err) + c->CoefB) / c->CoefA;
                else
                    next.ITerm = 0.0f;
            }
        }
        if (c->Improve & PID_Derivative_On_Measurement)
            next.Dout = c->Kd * (next.Last_Measure - next.Measure) / dt_s;
        if (!PIDStateIsFinite(&next))
            return PID_NUMERIC_ERROR;
        if (c->Improve & PID_DerivativeFilter)
        {
            float denominator = c->Derivative_LPF_RC + dt_s;
            float current = next.Dout * dt_s;
            float previous = next.Last_Dout * c->Derivative_LPF_RC;
            if (!isfinite(denominator) || !isfinite(current) || !isfinite(previous))
                return PID_NUMERIC_ERROR;
            next.Dout = current / denominator + previous / denominator;
        }
        if ((c->Improve & PID_Integral_Limit) && !PIDLimitIntegral(&next, c))
            return PID_NUMERIC_ERROR;
        next.Iout += next.ITerm;
        next.Output = next.Pout + next.Iout + next.Dout;
        if (!PIDStateIsFinite(&next))
            return PID_NUMERIC_ERROR;
        if (c->Improve & PID_OutputFilter)
        {
            float denominator = c->Output_LPF_RC + dt_s;
            float current = next.Output * dt_s;
            float previous = next.Last_Output * c->Output_LPF_RC;
            if (!isfinite(denominator) || !isfinite(current) || !isfinite(previous))
                return PID_NUMERIC_ERROR;
            next.Output = current / denominator + previous / denominator;
        }
        // 先检查溢出再限制正常结果，避免把 Inf 当成成功输出。
        if (!isfinite(next.Output))
            return PID_NUMERIC_ERROR;
        if (next.Output > c->MaxOut)
            next.Output = c->MaxOut;
        if (next.Output < -c->MaxOut)
            next.Output = -c->MaxOut;
    }
    else
    {
        // 沿用原死区行为，清除输出和本次、累计积分。
        next.Output = 0.0f;
        next.ITerm = 0.0f;
        next.Iout = 0.0f;
    }
    next.Last_Measure = next.Measure;
    next.Last_Output = next.Output;
    next.Last_Dout = next.Dout;
    next.Last_Err = next.Err;
    next.Last_ITerm = next.ITerm;
    if (!PIDStateIsFinite(&next))
        return PID_NUMERIC_ERROR;
    pid->state = next;
    *output = next.Output;
    return PID_OK;
}
