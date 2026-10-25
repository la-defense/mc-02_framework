/** @file task_monitor.h
 * @brief 配置周期／事件任务的期限监视，并为核心任务故障关断输出和停喂看门狗。
 * @note 初始化须在调度器启动前完成；各任务在普通任务上下文报告工作进度。
 */
#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>

typedef enum
{
    TASK_MONITOR_INS = 0,
    TASK_MONITOR_MOTOR,
    TASK_MONITOR_ROBOT,
    TASK_MONITOR_DAEMON,
    TASK_MONITOR_UART_TX,
    TASK_MONITOR_COUNT,
    TASK_MONITOR_OPTIONAL_0 = TASK_MONITOR_COUNT,
    TASK_MONITOR_OPTIONAL_1,
    TASK_MONITOR_OPTIONAL_2,
    TASK_MONITOR_ID_LIMIT
} TaskMonitor_Id_e;

#define TASK_MONITOR_MAX_TASKS ((uint32_t)TASK_MONITOR_ID_LIMIT)

typedef enum
{
    TASK_MONITOR_MODE_PERIODIC = 0,
    TASK_MONITOR_MODE_EVENT
} TaskMonitor_Mode_e;

typedef enum
{
    TASK_MONITOR_ROLE_CORE = 0,
    TASK_MONITOR_ROLE_AUXILIARY
} TaskMonitor_Role_e;

typedef enum
{
    TASK_MONITOR_WAIT_NONE = 0,
    TASK_MONITOR_WAIT_PERIOD,
    TASK_MONITOR_WAIT_EVENT,
    TASK_MONITOR_WAIT_SOURCE,
    TASK_MONITOR_WAIT_RESOURCE,
    TASK_MONITOR_WAIT_DELAY
} TaskMonitor_WaitReason_e;

typedef enum
{
    TASK_MONITOR_STATE_NOT_STARTED = 0,
    TASK_MONITOR_STATE_RUNNING,
    TASK_MONITOR_STATE_WAITING,
    TASK_MONITOR_STATE_COMPLETE,
    TASK_MONITOR_STATE_FAULTED
} TaskMonitor_State_e;

typedef enum
{
    TASK_MONITOR_FAULT_NONE = 0,
    TASK_MONITOR_FAULT_STARTUP_DEADLINE,
    TASK_MONITOR_FAULT_WORK_DEADLINE,
    TASK_MONITOR_FAULT_PERIOD_DEADLINE
} TaskMonitor_FaultReason_e;

typedef enum
{
    TASK_MONITOR_RESULT_OK = 0,
    TASK_MONITOR_RESULT_BAD_ARGUMENT,
    TASK_MONITOR_RESULT_BAD_CONFIGURATION,
    TASK_MONITOR_RESULT_NOT_INITIALIZED,
    TASK_MONITOR_RESULT_BUSY,
    TASK_MONITOR_RESULT_NOT_RUNNING,
    TASK_MONITOR_RESULT_CONTEXT,
    TASK_MONITOR_RESULT_SCHEDULER_RUNNING
} TaskMonitor_Result_e;

typedef enum
{
    TASK_MONITOR_LONG_OPERATION_CALIBRATION = 0,
    TASK_MONITOR_LONG_OPERATION_FLASH,
    TASK_MONITOR_LONG_OPERATION_COUNT
} TaskMonitor_LongOperation_e;

typedef void (*TaskMonitor_DegradeCallback_t)(TaskMonitor_Id_e id,
                                               uint32_t dependency_mask,
                                               void *context);

/** @brief 启动前固定的一项任务监控配置。 */
typedef struct
{
    TaskMonitor_Id_e id;                      /**< 配置表内唯一的任务编号。 */
    const char *name;                          /**< 稳定名称；配置生效期间保持可读。 */
    TaskMonitor_Mode_e mode;                   /**< 周期任务或按需唤醒的事件任务。 */
    TaskMonitor_Role_e role;                   /**< 核心故障关断输出，辅助故障只降级依赖。 */
    uint32_t period_ms;                        /**< 周期任务名义周期；事件任务必须为0。 */
    uint32_t deadline_ms;                      /**< 工作或下一轮完成期限，单位ms。 */
    uint32_t startup_grace_ms;                 /**< 首次报告前宽限，单位ms。 */
    uint32_t dependency_mask;                  /**< 辅助任务故障时置位的依赖位。 */
    TaskMonitor_DegradeCallback_t degrade_callback; /**< 辅助故障后调用，可为空。 */
    void *callback_context;                    /**< 回调上下文；配置生效期间保持有效。 */
} TaskMonitorTaskConfig_t;

/** @brief 任务进度的只读副本，所有时刻均为HAL毫秒时基。 */
typedef struct
{
    const char *name;
    uint8_t alive;
    uint8_t fault;
    uint32_t timeout_ms;
} TaskMonitorStatus_t;

typedef struct
{
    TaskMonitor_Mode_e mode;
    TaskMonitor_Role_e role;
    TaskMonitor_State_e state;
    TaskMonitor_FaultReason_e fault_reason;
    TaskMonitor_WaitReason_e wait_reason;
    uint32_t job_started_ms;
    uint32_t last_progress_ms;
    uint32_t waiting_since_ms;
    uint32_t last_completed_ms;
    uint32_t period_ms;
    uint32_t deadline_ms;
    uint32_t startup_grace_ms;
    uint32_t begin_count;
    uint32_t progress_count;
    uint32_t complete_count;
    uint32_t progress_value;
    uint32_t dependency_mask;
    uint8_t started;
    uint8_t work_active;
    uint8_t has_completed;
    uint8_t configured;
} TaskMonitorProgressState_t;

/**
 * @brief 使用内置五个周期核心任务配置，开始本次启动的监控。
 * @return 初始化成功返回 1；调度器已运行或调用上下文不符时返回 0。
 * @note 只能在创建／启动任务前调用。此时重复初始化会清除进度和故障记录；
 *       运行后不允许重置，避免清掉已经锁存的核心故障。
 *       需要自定义事件任务时使用 TaskMonitorInitConfigured()。
 */
uint8_t TaskMonitorInit(void);

/**
 * @brief 使用调用者提供的固定配置表初始化监控。
 * @param[in] configs 配置数组；名称及回调上下文须在本次启动期间保持有效。
 * @param[in] count 配置项数，范围为 1～TASK_MONITOR_MAX_TASKS。
 * @return 配置、参数、上下文或调度状态；失败时保留原有效监控配置。
 * @note 只能在调度器启动前调用。周期任务要求
 *       1 <= period_ms <= deadline_ms < INT32_MAX；事件任务的 period_ms 必须为 0。
 *       deadline_ms 和 startup_grace_ms 单位为毫秒，采用无符号时差检查，等于边界仍允许。
 */
TaskMonitor_Result_e TaskMonitorInitConfigured(const TaskMonitorTaskConfig_t *configs,
                                               uint32_t count);

/**
 * @brief 开始一轮周期工作或一个事件任务。
 * @param[in] id 已配置任务编号。
 * @return 成功或明确错误；核心故障锁存后不再启动新的核心工作。
 * @note 工作开始时记录固定总期限起点；后续进度报告不会延长此期限。
 */
TaskMonitor_Result_e TaskMonitorBeginWork(TaskMonitor_Id_e id);

/** @brief 记录当前工作步骤，不改变开始时刻或总期限。
 * @param[in] id 正在执行工作的任务编号。
 * @param[in] progress_value 调用方定义的阶段号或已完成计数。
 * @return 成功或明确错误；失败不改写监控状态。
 */
TaskMonitor_Result_e TaskMonitorReportProgress(TaskMonitor_Id_e id,
                                               uint32_t progress_value);

/**
 * @brief 记录任务当前等待的原因，不延长已开始工作的总期限。
 * @param[in] id 已配置任务编号；事件任务可在没有待处理事件时调用。
 * @param[in] reason 非 TASK_MONITOR_WAIT_NONE 的等待原因。
 * @return 成功或明确错误；非法原因不改写状态。
 */
TaskMonitor_Result_e TaskMonitorSetWaiting(TaskMonitor_Id_e id,
                                           TaskMonitor_WaitReason_e reason);

/** @brief 结束一轮工作或事件处理并记录完成时刻。
 * @param[in] id 已开始工作的任务编号。
 * @return 成功；若总期限已超，锁存对应任务故障并返回 BUSY；未开始时返回错误。
 */
TaskMonitor_Result_e TaskMonitorCompleteWork(TaskMonitor_Id_e id);

/**
 * @brief 读取一项任务的开始、进度、等待和完成记录。
 * @param[in] id 已配置任务编号。
 * @param[out] output 接收复制结果，包含等待开始时刻；空指针或读取失败时保持原值。
 * @return 成功或明确错误；查询不推进任务、不续期。
 */
TaskMonitor_Result_e TaskMonitorGetProgressState(TaskMonitor_Id_e id,
                                                 TaskMonitorProgressState_t *output);

/** @brief 复制当前所有辅助任务的依赖降级位。
 * @param[out] output 接收按配置依赖位或运算的结果；失败时保持原值。
 * @return 成功或明确错误。
 * @note 依赖位由配置定义；核心健康与来源在线／数据有效状态分别维护。
 */
TaskMonitor_Result_e TaskMonitorGetDegradedDependencies(uint32_t *output);

/** @brief 记录任务仍在运行；用于启动代码和已有的有界阻塞等待。
 * @param[in] id 有效任务编号；未配置或越界时忽略。
 * @note 工作已开始后，此接口只更新时间戳，不改变开始时刻或任务总期限。
 */
void TaskMonitorFeed(TaskMonitor_Id_e id);

/** @brief 等待指定毫秒，并记录等待原因。
 * @param[in] id 调用方任务编号，须来自 TaskMonitor_Id_e。
 * @param[in] milliseconds 等待时长，单位为 ms；为 0 不处理。
 * @note 任务运行时分成 1 ms 等待，启动前使用 DWT。等待与进度记录不会延长总期限。
 */
void TaskMonitorDelayMs(TaskMonitor_Id_e id, uint32_t milliseconds);

/** @brief 检查所有配置任务、报告核心健康并决定是否喂看门狗。
 * @note 由 Daemon 约每 10 ms 调用。核心故障会关断输出并锁存到本次启动结束；
 *       辅助故障只报告其依赖位并调用配置的降级回调。
 */
void TaskMonitorTick(void);

/** @brief 为 INS 申请有时限的标定或 Flash 操作窗口。
 * @param[in] id 目前只允许 TASK_MONITOR_INS。
 * @param[in] operation 标定最多 15000 ms，Flash 保存最多 8000 ms。
 * @return 1 为开始成功，0 为未初始化、已有核心故障、重复申请或条件不符。
 * @note 标定只暂缓 INS 的常规期限；Flash 暂缓所有常规期限。窗口自身期限不续期。
 */
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id,
                                      TaskMonitor_LongOperation_e operation);

/** @brief 结束对应长操作并检查总耗时及核心任务。
 * @param[in] id 须为先前申请的 INS。
 * @param[in] operation 须为已开始且尚未结束的操作类型。
 * @return 1 为按时结束且核心任务正常；0 为申请不匹配、超时或发生故障。
 * @note 按时结束标定只更新 INS 记录，按时结束 Flash 更新所有周期核心任务记录。
 */
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id,
                                    TaskMonitor_LongOperation_e operation);

/** @brief 读取已保存的核心任务正常状态，不在此处重新检查当前时间。
 * @return 1 为所有核心任务健康且没有锁存故障，否则为 0。
 */
uint8_t TaskMonitorAllAlive(void);

/**
 * @brief 按当前 HAL tick 再检查全部核心任务的进度期限。
 * @return 1 表示至少有一项核心任务且当前未过期；未初始化、无核心项或已过期时返回0。
 * @note 供任务上下文的稳定期判断使用；不会关断输出或发布故障动作。
 *       检查到标定／Flash窗口自身超时时会锁存该超时，后续监控周期负责发布关断动作。
 */
uint8_t TaskMonitorAllCoreHealthyNow(void);

/** @brief 读取指定任务的最近监控结果。
 * @param[in] id 已配置任务编号。
 * @param[out] status 接收任务名、健康标记和期限；参数错误时保持输出不变。
 */
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status);

#endif
