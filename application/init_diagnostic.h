/**
 * @file init_diagnostic.h
 * @brief 记录启动进度和首次失败，供程序及调试器查询。
 * RobotInit() 更新阶段，各初始化入口报告失败；INS 初始化成功后标记 READY。
 * 记录在每次启动时从零开始，以下接口不会清除已记录的失败。
 */
#ifndef MC02_INIT_DIAGNOSTIC_H
#define MC02_INIT_DIAGNOSTIC_H

#include <stdint.h>

#define INIT_DIAGNOSTIC_EVENT_CAPACITY 16u

/** @brief 启动阶段；发生失败后保留当时的阶段。 */
typedef enum
{
    INIT_STAGE_RESET = 0,    ///< 尚未进入板级初始化
    INIT_STAGE_BSP,          ///< 板级外设初始化
    INIT_STAGE_APPLICATION,  ///< 业务模块初始化
    INIT_STAGE_TASKS,        ///< 创建机器人任务
    INIT_STAGE_FREERTOS,     ///< 创建默认任务并启动调度器
    INIT_STAGE_READY,        ///< INS 初始化已完成
} InitDiagnosticStage_e;

/** @brief 初始化失败的原因编号。 */
typedef enum
{
    INIT_FAILURE_NONE = 0,            ///< 尚未记录失败
    INIT_FAILURE_RESOURCE,            ///< 资源申请或注册失败
    INIT_FAILURE_TASK_CREATE,         ///< 任务创建失败
    INIT_FAILURE_SCHEDULER_RETURNED,  ///< 启动调度器的调用返回错误
    INIT_FAILURE_ALGORITHM_MEMORY,    ///< 算法初始化所需内存申请失败
    INIT_FAILURE_DEPENDENCY,          ///< 依赖模块未能完成初始化
} InitDiagnosticReason_e;

/** @brief 失败所属模块的编号；0 表示尚未记录模块。 */
typedef enum
{
    INIT_MODULE_ROBOT = 1,
    INIT_MODULE_REMOTE,
    INIT_MODULE_VISION,
    INIT_MODULE_GIMBAL,
    INIT_MODULE_SHOOTER,
    INIT_MODULE_CHASSIS,
    INIT_MODULE_INS,  ///< 姿态解算模块
    INIT_MODULE_TASKS,
    INIT_MODULE_FREERTOS,
    INIT_MODULE_BUZZER,
} InitDiagnosticModule_e;

/** @brief 任务创建失败时使用的资源编号，便于找到具体任务。 */
typedef enum
{
    INIT_RESOURCE_TASK_INS = 1,
    INIT_RESOURCE_TASK_MOTOR,
    INIT_RESOURCE_TASK_DAEMON,
    INIT_RESOURCE_TASK_ROBOT,
    INIT_RESOURCE_TASK_UI,
    INIT_RESOURCE_TASK_LCD,
    INIT_RESOURCE_TASK_DEFAULT,
    INIT_RESOURCE_TASK_UART_TX,
    INIT_RESOURCE_TASK_LOG,
} InitDiagnosticResource_e;

/** @brief 本次启动的诊断记录；调试器按这些字段的类型和顺序读取。 */
typedef struct
{
    uint8_t failed;          ///< 0：尚无失败；1：已记录失败
    uint8_t stage;           ///< 当前阶段，失败后保留首次失败阶段，取 InitDiagnosticStage_e
    uint8_t module;          ///< 首次失败的模块编号，取 InitDiagnosticModule_e
    uint8_t reason;          ///< 首次失败的原因编号，取 InitDiagnosticReason_e
    uint16_t resource_id;   ///< 首次失败的具体资源编号，由报告失败的代码指定
    uint16_t reserved;      ///< 预留字段，保持启动时的 0
    uint32_t failure_count; ///< 报告失败的调用次数，到 UINT32_MAX 后保持不变
    uint32_t heartbeat;     ///< 心跳接口的调用次数，单位为次，超过 UINT32_MAX 后回到 0
} InitDiagnosticSnapshot_s;

/** @brief 初始化历史事件的种类。 */
typedef enum
{
    INIT_DIAGNOSTIC_EVENT_STAGE_CHANGED = 1u,
    INIT_DIAGNOSTIC_EVENT_FAILURE,
    INIT_DIAGNOSTIC_EVENT_READY,
    INIT_DIAGNOSTIC_EVENT_DEGRADED,
} InitDiagnosticEventKind_e;

/** @brief 初始化历史事件的严重程度。 */
typedef enum
{
    INIT_DIAGNOSTIC_SEVERITY_INFO = 1u,
    INIT_DIAGNOSTIC_SEVERITY_ERROR,
    INIT_DIAGNOSTIC_SEVERITY_WARNING,
} InitDiagnosticSeverity_e;

/** @brief 一次阶段变化、初始化失败或 READY 事件；时间为本机 HAL 毫秒，0合法。 */
typedef struct
{
    uint32_t sequence;   ///< 非0事件序号，递增到上限后回到1
    uint32_t timestamp_ms; ///< 事件发生时刻，单位 ms
    uint16_t resource_id; ///< 与 InitDiagnosticSnapshot_s 相同的资源编号
    uint8_t kind;       ///< InitDiagnosticEventKind_e
    uint8_t stage;      ///< InitDiagnosticStage_e
    uint8_t module;     ///< InitDiagnosticModule_e；阶段事件为0
    uint8_t reason;     ///< InitDiagnosticReason_e；非失败事件为 NONE
    uint8_t severity;   ///< InitDiagnosticSeverity_e
} InitDiagnosticEvent_t;

/** @brief 最近最多16条初始化事件，按发生先后从旧到新排列。 */
typedef struct
{
    uint32_t total_count; ///< 本次启动累计事件数，到 UINT32_MAX 后保持
    uint8_t count;        ///< 本次复制的条目数，最多 INIT_DIAGNOSTIC_EVENT_CAPACITY
    InitDiagnosticEvent_t events[INIT_DIAGNOSTIC_EVENT_CAPACITY];
} InitDiagnosticEventSnapshot_t;

/** @brief 供调试器通过 SWD 只读查看；程序中用 InitDiagnosticGetSnapshot() 复制记录。 */
extern volatile InitDiagnosticSnapshot_s init_diagnostic_snapshot;

/**
 * @brief 尚无失败时记录当前启动阶段。
 * @param[in] stage 当前进入的阶段，使用 InitDiagnosticStage_e 中的值。
 * @note 只更新阶段，不清除失败标志或计数；已记录失败时保持原阶段。
 */
void InitDiagnosticBegin(InitDiagnosticStage_e stage);

/**
 * @brief 更新启动阶段，用法及失败后的处理与 InitDiagnosticBegin() 相同。
 * @param[in] stage 当前进入的阶段，使用 InitDiagnosticStage_e 中的值。
 */
void InitDiagnosticSetStage(InitDiagnosticStage_e stage);

/**
 * @brief 记录首次初始化失败，并累计报告失败的次数。
 * @param[in] stage 失败发生的阶段，使用 InitDiagnosticStage_e 中的值。
 * @param[in] module 失败所属模块，使用 InitDiagnosticModule_e 中的值。
 * @param[in] reason 失败原因，使用 InitDiagnosticReason_e 中的值。
 * @param[in] resource_id 具体资源编号；任务创建使用 InitDiagnosticResource_e，其他资源由调用方编号。
 * @note 后续报告只增加 failure_count，不覆盖首次失败的四项信息。
 *       failure_count 到 UINT32_MAX 后保持不变；本函数只记录，后续停止控制等处理由调用方负责。
 */
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource_id);

/** @brief 记录辅助模块失败；不锁存整车失败，不改变首次核心失败或阶段。
 * @param[in] stage 发生时的启动阶段。
 * @param[in] module 失败模块编号。
 * @param[in] reason 具体失败原因。
 * @param[in] resource_id 具体资源编号。
 * @note 固定容量事件写入；不分配、不等待，也不解除已有失能。
 */
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource_id);

/**
 * @brief 尚无失败时将诊断阶段设为 INIT_STAGE_READY。
 * @note 当前由 INS 初始化成功后调用；实际是否允许控制由 RobotSafety 处理。
 */
void InitDiagnosticMarkReady(void);

/**
 * @brief 将 heartbeat 加 1，供观察诊断代码是否仍在运行。
 * @note 失败后仍会计数；它记录调用次数，不代表经过的毫秒数或机器人已就绪。
 */
void InitDiagnosticHeartbeat(void);

/**
 * @brief 查询本次启动是否已记录失败，不清除记录。
 * @return 0 表示尚无失败，1 表示已记录失败。
 */
uint8_t InitDiagnosticIsFailed(void);

/**
 * @brief 将当前诊断记录复制到调用方提供的位置，不清除原记录。
 * @param[out] snapshot 用于保存完整记录的结构体；为 NULL 时直接返回。
 * @note 复制时暂时关闭普通中断，结束后恢复调用前的中断开关状态。
 */
void InitDiagnosticGetSnapshot(InitDiagnosticSnapshot_s *snapshot);

/**
 * @brief 将最近的初始化事件复制到调用方缓冲区，不消费记录。
 * @param[out] snapshot 接收事件，不能为NULL。
 * @return 1表示复制成功；NULL 返回0，输出不可用时原值保持。
 * @note 事件最多保存16条，满后覆盖最旧项；查询在短临界区内复制，不分配内存。
 */
uint8_t InitDiagnosticGetEvents(InitDiagnosticEventSnapshot_t *snapshot);

#endif
