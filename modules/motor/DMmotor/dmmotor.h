/**
 * @file dmmotor.h
 * @brief DM 直接力矩模式的注册、任务命令和状态读取接口。
 * @note 当前仅支持 CAN 标准帧下的直接力矩控制；MIT 闭环扩展由 MC02-0073 跟踪。
 *       运行期接口只从普通任务调用，不得从中断调用。
 */
#ifndef MC02_DMMOTOR_H
#define MC02_DMMOTOR_H

#include <stdint.h>
#include "motor_def.h"

#define DM_MOTOR_CNT 4u

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX  18.0f

typedef struct DMMotorInstance DMMotorInstance;

/** @brief 区分参数、配置、就绪、队列和资源结果；QUEUED 不代表 CAN 已接受。 */
typedef enum
{
    DMMOTOR_STATUS_OK = 0,
    DMMOTOR_STATUS_QUEUED,
    DMMOTOR_STATUS_CANCELLED,
    DMMOTOR_STATUS_INVALID_PARAMETER,
    DMMOTOR_STATUS_INVALID_CONFIG,
    DMMOTOR_STATUS_NOT_REGISTERED,
    DMMOTOR_STATUS_NOT_READY,
    DMMOTOR_STATUS_BUSY,
    DMMOTOR_STATUS_RESOURCE_ERROR,
    DMMOTOR_STATUS_INVALID_VALUE,
} DMMotorStatus_t;

/** @brief DM 任务可提交的显式协议命令。 */
typedef enum
{
    DMMOTOR_COMMAND_NONE = 0,
    DMMOTOR_COMMAND_ENABLE,
    DMMOTOR_COMMAND_ZERO_POSITION,
} DMMotorCommandType_t;

/** @brief 最近命令结果；ACCEPTED 仅表示 CAN 接口接受报文，不是电机执行确认。 */
typedef enum
{
    DMMOTOR_COMMAND_RESULT_NONE = 0,
    DMMOTOR_COMMAND_PENDING,
    DMMOTOR_COMMAND_ACCEPTED,
    DMMOTOR_COMMAND_REJECTED,
    DMMOTOR_COMMAND_CANCELLED,
} DMMotorCommandResult_t;

/** @brief 最近一轮控制输入检查结果；无参考时发送零力矩，不复用旧目标。 */
typedef enum
{
    DMMOTOR_CALCULATION_NO_REFERENCE = 0,
    DMMOTOR_CALCULATION_OK,
    DMMOTOR_CALCULATION_INVALID_INPUT,
} DMMotorCalculationStatus_t;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,
    DM_CMD_RESET_MODE = 0xfd,
    DM_CMD_ZERO_POSITION = 0xfe,
    DM_CMD_CLEAR_ERROR = 0xfb,
} DMMotor_Mode_e;

/** @brief 查询到的实例状态；该副本可修改，不会改变电机实例。 */
typedef struct
{
    uint8_t task_ready; /**< 1 表示全部 DM 任务已创建并可处理命令。 */
    uint8_t running; /**< 1 表示本地正在采用参考；停止后仍可能周期发送零力矩。 */
    uint8_t protocol_enabled; /**< 1 表示 DM 使能命令已被 CAN 接受。 */
    uint8_t reference_available; /**< 1 表示存在有限的新参考值。 */
    uint32_t command_sequence; /**< 最近命令序号；从1递增，回绕时跳过0。 */
    DMMotorCommandType_t command_type; /**< 最近命令类型。 */
    DMMotorCommandResult_t command_result; /**< 最近命令排队、接受、拒绝或取消状态。 */
    DMMotorCalculationStatus_t calculation_status; /**< 最近一轮力矩计算结果。 */
} DMMotorState_t;

/**
 * @brief 注册 DM 电机并准备 CAN 与 Daemon 资源。
 * @param[in] config 初始化配置；标准 CAN 收发 ID 必须在0～0x7FF，同总线发送 ID 或接收 ID 不可重复，
 *            最多注册4个实例。方向只能为正向或反向，闭环须为 OPEN_LOOP 且前馈须关闭；PID 与反馈字段不使用，可为空。
 * @return 成功返回长期有效实例；配置错误、未知 CAN、重复 ID、容量或资源失败返回 NULL，配置不会被改写。
 * @note 限调度器启动前串行调用。成功后配置可释放，CAN 句柄须长期有效。
 *       初始化不发送报文、不等待，也不使能电机。任务启动后禁止继续注册。
 */
DMMotorInstance *DMMotorInit(const Motor_Init_Config_s *config);

/**
 * @brief 保存有限力矩参考。
 * @param[in] motor 已注册实例；实例由模块长期保存。
 * @param[in] ref 有限的协议数值，单位尚未按具体型号／固件标定；超出−18～18时发送前限幅，有限输入仍返回 OK。
 * @return DMMOTOR_STATUS_OK；空参数、实例未注册或 ref 为 NaN／Inf 时返回对应状态。
 * @note 非有限 ref 会清除旧参考并保存零，避免后续继续使用过期目标。使能命令被 CAN 接受时
 *       会清除所有此前参考；调用方应查询命令状态后再提供新参考。
 */
DMMotorStatus_t DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 检查控制模式请求。
 * @param[in] motor 已注册实例。
 * @param[in] type 本轮只接受 OPEN_LOOP 直接力矩模式。
 * @return 支持时返回 DMMOTOR_STATUS_OK；其他闭环配置返回 DMMOTOR_STATUS_INVALID_CONFIG。
 */
DMMotorStatus_t DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type);

/**
 * @brief 请求使能 DM 电机。
 * @param[in] motor 已注册且 DM 控制任务已就绪的实例。
 * @return 新命令返回 DMMOTOR_STATUS_QUEUED；已有命令时返回 BUSY；未就绪或非法实例返回错误。
 * @note 命令由 DM 任务尝试发送一次。返回 QUEUED 不表示 CAN 已接受或电机已执行。
 *       已处于本地运行状态时返回 OK，不重复发送使能命令。
 */
DMMotorStatus_t DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 停止采用参考并清除旧目标。
 * @param[in] motor 已注册实例。
 * @return 成功返回 OK；尚未提交的命令被取消时返回 CANCELLED；非法实例返回错误。
 * @note 已使能的协议状态继续每轮发送零力矩帧以保留反馈；不发送退出模式命令。
 *       已进入 CAN 提交的报文不能撤回，提交结束后也不会重新打开本地运行状态。
 */
DMMotorStatus_t DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 请求将当前位置设为编码器零位。
 * @param[in] motor 已注册、任务就绪且本地停止的实例。
 * @return 命令排队返回 QUEUED；运行中返回 NOT_READY；命令槽占用返回 BUSY。
 * @note 命令由任务发送一次，发送结果仅表示本机 CAN 接受状态。调用方负责确认机械静止。
 *       置零命令不等待、不使能电机，置零后仍保持停止。
 */
DMMotorStatus_t DMMotorCaliEncoder(DMMotorInstance *motor);

/**
 * @brief 为已注册电机创建独立任务。
 * @return 全部任务创建成功或已启动时返回 OK；无实例、资源隔离或创建／回收错误返回明确状态。
 * @note 只可在调度器启动前串行调用。任务保留普通优先级、128字节请求栈和约2ms循环周期。
 *       创建失败会回收本次已创建任务；无法确认回收时模块隔离，须重启系统恢复。
 */
DMMotorStatus_t DMMotorControlInit(void);

/**
 * @brief 复制实例最近状态。
 * @param[in] motor 已注册实例。
 * @param[out] state 输出结构；失败时原内容保持不变。
 * @return 成功返回 OK；空参数或未注册实例返回对应错误。
 * @note 查询不推进命令、不发送报文，也不修改实例。
 */
DMMotorStatus_t DMMotorGetState(const DMMotorInstance *motor, DMMotorState_t *state);

#endif /* MC02_DMMOTOR_H */
