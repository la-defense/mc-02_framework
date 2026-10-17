/**
 * @file controller.h
 * @brief 声明由调用者保存状态、显式传入秒单位间隔的位置式 PID。
 * 首次使用先将实例清零，再调用 PIDInit；同一实例由一个调用者顺序使用。
 * @author Wang Hongxi
 * @version V1.1.3
 * @date 2021/7/3
 */
#ifndef MC02_CONTROLLER_H
#define MC02_CONTROLLER_H
#include <stdint.h>

/** @brief 可组合的改进选项；比例先行只保留编号，初始化时拒绝选用。 */
typedef enum {
    PID_IMPROVE_NONE = 0x00,
    PID_Integral_Limit = 0x01,
    PID_Derivative_On_Measurement = 0x02,
    PID_Trapezoid_Intergral = 0x04,
    PID_Proportional_On_Measurement = 0x08,
    PID_OutputFilter = 0x10,
    PID_ChangingIntegrationRate = 0x20,
    PID_DerivativeFilter = 0x40,
    PID_ErrorHandle = 0x80,
} PID_Improvement_e;

/** @brief 接口执行结果；调用者只能在 PID_OK 时采用本次计算输出。 */
typedef enum {
    PID_OK = 0,
    PID_BAD_ARGUMENT,
    PID_BAD_CONFIG,
    PID_NOT_INITIALIZED,
    PID_BAD_DT,
    PID_NUMERIC_ERROR,
    PID_UNSUPPORTED_IMPROVEMENT,
} PID_Status_e;

typedef enum errorType_e {
    PID_ERROR_NONE = 0,
    PID_MOTOR_BLOCKED_ERROR = 1,
} ErrorType_e;

/** @brief 堵转检查按成功计算的调用次数累计，超过500次后记录错误。 */
typedef struct {
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/** @brief PID 参数；measure 与 ref 单位相同，增益决定输出单位。 */
typedef struct {
    float Kp, Ki, Kd;
    float MaxOut;            // 输出绝对值上限，允许为0。
    float DeadBand;          // 误差不超过此值时清除输出和积分。
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分绝对值上限，允许为0。
    float CoefA;             // 变速积分过渡宽度，启用该选项时须大于0。
    float CoefB;             // 变速积分的完整积分区间，须不小于0。
    float Output_LPF_RC;     // 输出滤波时间，单位 s，允许为0。
    float Derivative_LPF_RC; // 微分滤波时间，单位 s，允许为0。
    float DtMin_s;           // 最小间隔，单位 s，须大于0。
    float DtMax_s;           // 最大间隔，单位 s，须不小于 DtMin_s。
} PID_Init_Config_s;

/** @brief 最近一次成功计算的数据；通过 PIDGetState 复制后读取。 */
typedef struct {
    float Measure, Last_Measure;
    float Err, Last_Err, Last_ITerm;
    float Pout, Iout, Dout, ITerm;
    float Output, Last_Output, Last_Dout;
    float Ref;
    float dt; // 最近成功计算使用的秒单位间隔。
    PID_ErrorHandler_t ERRORHandler;
} PID_State_t;

/** @brief 调用者提供长期存储；初始化后只经下列接口修改内容。 */
typedef struct {
    PID_Init_Config_s config;
    PID_State_t state;
    uint32_t initialized;
} PIDInstance;

/**
 * @brief 检查参数并清除运行历史，成功后接受整份配置。
 * @param[out] pid 首次使用前清零的实例，不能为 NULL。
 * @param[in] config 参数均须有限；上限、死区、时间参数及 CoefB 不小于0。
 * @return PID_OK 表示成功；空参数、非法配置或未实现选项返回对应错误。
 * @note 失败不修改实例；config 可指向实例内原配置。
 */
PID_Status_e PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);

/**
 * @brief 用反馈、目标和真实间隔计算本次输出。
 * @param[in,out] pid 已初始化实例，同一实例不可并发修改。
 * @param[in] measure 有限反馈值，与 ref 使用相同单位。
 * @param[in] ref 有限目标值。
 * @param[in] dt_s 两次控制计算之间的秒数，允许等于配置上下限。
 * @param[out] output 本次输出，不能为 NULL 或与实例存储重叠。
 * @return PID_OK 表示成功；失败时实例和 output 均保持原值。
 * @note 不读取时钟、不分配内存；失败后关闭实际输出由调用模块负责。
 */
PID_Status_e PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s, float *output);

/**
 * @brief 清除积分、微分、输出及堵转历史，保留初始化参数。
 * @param[in,out] pid 已初始化实例，不能为 NULL。
 * @return PID_OK 表示成功；空参数或未初始化时返回错误且保持原值。
 */
PID_Status_e PIDReset(PIDInstance *pid);

/**
 * @brief 复制最近一次成功计算的数据，读取副本不会修改 PID。
 * @param[in] pid 已初始化实例，不能为 NULL。
 * @param[out] state 独立接收对象，不能为 NULL 或与实例存储重叠。
 * @return PID_OK 表示成功；失败时 state 保持原值。
 * @note 读取期间同一实例不得正在被其他任务修改。
 */
PID_Status_e PIDGetState(const PIDInstance *pid, PID_State_t *state);
#endif
