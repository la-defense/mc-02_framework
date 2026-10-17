/**
* @file controller.h
* @brief 声明位置式 PID 控制器、参数和可选改进功能。
* PIDInit 载入参数后，周期调用 PIDCalculate；反馈值和设定值须使用同一单位。
* @author Wang Hongxi
 * @version V1.1.3
* @date 2021/7/3
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "main.h"
#include "stdint.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
/**
* @brief 用位标志选择 PID 计算中的改进功能，可同时打开多个选项。
 */
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
/**
* @brief PID 每次计算使用的参数和保存的中间结果。
 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波时间，单位 s
    float Derivative_LPF_RC; // 微分滤波时间，单位 s

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
/**
* @brief 传给 PIDInit 的参数。MaxOut 限制最终输出，IntegralLimit 限制积分累计值。
* Output_LPF_RC 和 Derivative_LPF_RC 是滤波时间参数，单位 s。
 */
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // 输出滤波时间，单位 s
    float Derivative_LPF_RC; // 微分滤波时间，单位 s
} PID_Init_Config_s;

/**
* @brief 清零 PID 状态并复制初始化参数。
* @param pid 要初始化的 PID 实例。
* @param config 参数对象；两者都必须是有效指针。
* @note 不检查空指针或参数范围；调用前应保证配置值可用。
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
* @brief 用一组反馈值和设定值计算 PID 输出。
* @param pid 已初始化的 PID 实例。
* @param measure 当前反馈值。
* @param ref 当前设定值。
* @return 本次控制输出。输出单位由配置参数和控制对象决定。
* @note measure 与 ref 使用同一单位；本函数按秒更新积分和微分。
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif