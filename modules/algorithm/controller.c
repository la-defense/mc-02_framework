/**
 * @file controller.c
* @brief 实现位置式 PID、积分与微分改进、输出滤波和堵转计数。
* PID 参数和可选功能在 controller.h 中说明。
* @author Wang Hongxi
 * @author modified by neozng
 * @date 2022-11-01
* @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"
#include <math.h>

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    // 用本次和上次误差的平均值计算积分增量。
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        // 误差较小时保留完整积分；再增大时逐步减弱积分作用。
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    // 输出已接近上限且积分还在推高输出时，丢掉本次新增积分。
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0) // 积分却还在累积
        {
            pid->ITerm = 0; // 当前积分项置零
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /*Motor Blocked Handle*/
    // 输出很小或设定值接近0时不累计堵转次数。
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // 计数超过500后标记为堵转；计数单位是本函数被调用的次数。
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
* @brief 清零运行状态、复制初始化配置，并开始记录计算间隔。
* @param pid 已分配的 PID 实例。
* @param config 要复制的 PID 参数。
* 两个指针必须有效；本函数不检查空指针或参数范围。
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    memset(pid, 0, sizeof(PIDInstance));
    // 这里依赖两个结构体开头字段的类型和顺序一致；改动字段时要重新核对。
    memcpy(pid, config, sizeof(PID_Init_Config_s));
    // 记录本次时间，供下一次计算两次调用之间的间隔。
    DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
* @brief 根据当前反馈和设定值计算 PID 输出。
* @param pid 已初始化的 PID 实例。
* @param measure 当前反馈值。
* @param ref 当前设定值。
* @return 本次输出值；具体单位由控制对象和增益决定。
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    // 堵转检测
    // 堵转检查读取上一次保存的参考值、反馈值和输出。
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 取两次计算间隔，单位为秒。
    if (pid->dt < 1e-5f) // 间隔太小时使用最小值，避免微分除以接近零的时间。
        pid->dt = 1e-5f;

    // 用设定值减去反馈值，得到本次控制误差。
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    // 误差超过死区才计算比例、积分和微分项。
    if (fabsf(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 误差进入死区后清除输出和累计积分，避免退出死区时带着旧积分。
    {
        pid->Output = 0;
        pid->ITerm = 0;
        pid->Iout = 0;
    }

    // 保存本次结果，供下一次微分和输出滤波使用。
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
