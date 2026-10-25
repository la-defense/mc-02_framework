/** @file daemon.h
 * @brief 按绝对毫秒期限记录来源状态，并把状态变化交给唯一任务消费。
 * @note 注册只在启动阶段进行；数据上报可来自已校验的任务接收路径。
 */
#ifndef MC02_DAEMON_H
#define MC02_DAEMON_H

#include <stdint.h>

#define DAEMON_MX_CNT 64u
#define DAEMON_EVENT_MAX_DEPTH 8u
#define DAEMON_EVENT_TASK_CONSUMER 0xffu
#define DAEMON_CONSUMER_MOTOR_TASK 1u
#define DAEMON_CONSUMER_ROBOT_TASK 2u

typedef struct daemon_ins DaemonInstance;

typedef enum
{
    DAEMON_OK = 0,
    DAEMON_ERR_ARGUMENT,
    DAEMON_ERR_CONFIG,
    DAEMON_ERR_NOT_REGISTERED,
    DAEMON_ERR_CONSUMER,
    DAEMON_ERR_EMPTY,
    DAEMON_ERR_EXPIRED,
    DAEMON_ERR_CAPACITY,
    DAEMON_ERR_TASK_CREATE,
    DAEMON_NO_TASK_REQUIRED
} DaemonStatus_t;

typedef enum
{
    DAEMON_SOURCE_WAITING_FIRST_FRAME = 0,
    DAEMON_SOURCE_ONLINE_DATA_VALID,
    DAEMON_SOURCE_ONLINE_DATA_INVALID,
    DAEMON_SOURCE_OFFLINE
} DaemonSourceState_t;

typedef enum
{
    DAEMON_EVENT_FIFO = 1,
    DAEMON_EVENT_COUNT,
    DAEMON_EVENT_COALESCED_STATE
} DaemonEventPolicy_t;

typedef enum
{
    DAEMON_CONSUMER_BOUND_TASK = 0,
    DAEMON_CONSUMER_INDEPENDENT_TASK
} DaemonConsumerMode_t;

struct daemon_event;

typedef struct
{
    uint32_t transport_timeout_ms; /**< 合法通信的期限，范围1～INT32_MAX ms。 */
    uint32_t data_timeout_ms;      /**< 关键数据有效期限，范围1～INT32_MAX ms。 */
    uint32_t startup_grace_ms;     /**< 首帧等待宽限，范围0～INT32_MAX ms。 */
    uint32_t event_max_age_ms;     /**< 未消费事件的期限，范围1～INT32_MAX ms。 */
    uint16_t source_id;            /**< 调用方分配的稳定来源编号。 */
    uint8_t event_capacity;        /**< FIFO容量1～8；计数与合并策略使用1。 */
    uint8_t consumer_id;           /**< 绑定任务编号；独立任务模式忽略此字段。 */
    DaemonEventPolicy_t event_policy;
    DaemonConsumerMode_t consumer_mode;
    void (*event_handler)(const struct daemon_event *event, void *context);
    void *owner_id;                /**< 事件处理上下文，注册后由来源模块持有。 */
} Daemon_Init_Config_s;

typedef struct daemon_event
{
    uint16_t source_id;
    uint32_t sequence;
    uint32_t detected_ms;
    uint32_t last_detected_ms;
    uint32_t occurrences;
    DaemonSourceState_t previous_state;
    DaemonSourceState_t state;
} DaemonEvent_t;

typedef struct
{
    DaemonSourceState_t state;
    uint32_t last_transport_rx_ms;
    uint32_t last_data_rx_ms;
    uint32_t transport_age_ms;
    uint32_t data_age_ms;
    uint32_t transport_deadline_ms;
    uint32_t data_deadline_ms;
    uint8_t has_transport_frame;
    uint8_t has_data_sample;
    uint8_t transport_online;
    uint8_t data_valid;
} DaemonSourceStatus_t;

typedef struct
{
    uint32_t enqueued;
    uint32_t dispatched;
    uint32_t overflowed;
    uint32_t expired;
    uint16_t queued;
    uint8_t task_started;
    uint8_t task_failed;
    uint32_t global_overflowed;
    uint32_t global_expired;
} DaemonStats_t;

/* 上报标记：只有已通过协议校验的通信帧才能刷新传输期限。 */
#define DAEMON_REPORT_TRANSPORT_VALID (1u << 0)
#define DAEMON_REPORT_DATA_PRESENT    (1u << 1)
#define DAEMON_REPORT_DATA_VALID      (1u << 2)

typedef struct daemon_ins
{
    uint32_t transport_timeout_ms;
    uint32_t data_timeout_ms;
    uint32_t startup_grace_ms;
    uint32_t event_max_age_ms;
    uint32_t registered_ms;
    uint32_t last_transport_rx_ms;
    uint32_t last_data_rx_ms;
    uint32_t next_event_sequence;
    uint32_t enqueued;
    uint32_t dispatched;
    uint32_t overflowed;
    uint32_t expired;
    uint16_t source_id;
    uint8_t has_transport_frame;
    uint8_t has_data_sample;
    uint8_t data_valid;
    uint8_t consumer_id;
    uint8_t consumer_mode;
    uint8_t event_policy;
    uint8_t event_capacity;
    uint8_t event_head;
    uint8_t event_count;
    DaemonSourceState_t published_state;
    void (*event_handler)(const DaemonEvent_t *event, void *context);
    void *owner_id;
    DaemonEvent_t events[DAEMON_EVENT_MAX_DEPTH];
} DaemonInstance;

/** @brief 注册一个来源监控实例，复制配置并建立首帧等待期限。
 * @param[in] config 只读配置；期限单位ms，超时和事件期限为1～INT32_MAX，事件容量为1～8。
 * @return 成功返回长期实例；参数、配置、容量或分配失败返回NULL。
 * @note 只在调度器启动前串行调用。独立任务模式须提供事件处理函数，并在任务创建后调用
 *       DaemonEventTaskInit；绑定模式由配置指定的唯一任务调用事件处理接口。
 */
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config);

/** @brief 注销启动期注册的实例并释放存储。
 * @param[in] instance 由 DaemonRegister 返回、且不再被任何任务读取的实例。
 * @return 1表示已注销，0表示参数或注册状态错误。
 * @note 仅允许在监控任务及事件消费者均未运行时调用。
 */
uint8_t DaemonUnregister(DaemonInstance *instance);

/** @brief 上报一个接收结果；不合法的协议帧不会刷新通信期限。
 * @param[in] instance 已注册来源。
 * @param[in] received_ms 来源所在设备收到帧时的本机HAL毫秒时间；0合法。
 * @param[in] flags TRANSPORT_VALID表示协议合法；DATA_PRESENT表示本帧含关键数据；
 *        DATA_VALID仅在DATA_PRESENT时生效。
 * @return DAEMON_OK或具体参数、注册和旧时间错误；失败不改写原状态。
 * @note 无符号时差用于识别回绕；过期的乱序接收记录不覆盖更新记录。
 */
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags);

/** @brief 复制来源通信与关键数据状态。
 * @param[in] instance 已注册来源。
 * @param[in] now_ms 本机HAL毫秒时刻，0及回绕合法。
 * @param[out] output 接收状态副本；参数错误时保持原值。
 * @return DAEMON_OK或参数／注册错误。
 * @note 查询不刷新时间、不生成事件。通信超时与关键数据超时分别判定，等于期限仍有效。
 */
DaemonStatus_t DaemonGetSourceStatus(const DaemonInstance *instance, uint32_t now_ms,
                                     DaemonSourceStatus_t *output);

/** @brief 由监控任务检查所有来源的绝对期限，并只排入状态变化事件。
 * @note 此函数不调用模块事件处理函数；任务周期不会改变来源期限。
 */
void DaemonTask(void);

/** @brief 处理绑定到同一逻辑任务的来源事件。
 * @param[in] consumer_id Daemon_Init_Config_s.consumer_id中的任务编号。
 * @param[in] max_events 本次总处理上限，范围1～8。
 * @param[out] processed 可为空；成功时返回处理数量。
 * @return DAEMON_OK或参数错误；队列为空仍返回DAEMON_OK且数量为0。
 */
DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer_id, uint8_t max_events,
                                       uint8_t *processed);

/** @brief 创建可选独立事件任务；仅为独立模式来源分发事件。
 * @return DAEMON_OK、NO_TASK_REQUIRED或TASK_CREATE错误。
 * @note 由启动阶段调用一次；任务使用512个StackType_t，每1ms最多处理8条，回调须短小且不等待。
 */
DaemonStatus_t DaemonEventTaskInit(void);

/** @brief 查询是否登记了需要独立事件任务的来源；供启动适配层使用。 */
uint8_t DaemonHasIndependentSources(void);

/** @brief 更新独立事件任务的启动结果；仅由启动适配层调用。 */
void DaemonSetEventTaskResult(uint8_t started, uint8_t failed);

/** @brief 复制来源事件统计与独立任务创建状态。
 * @param[in] instance 来源实例；允许NULL以读取全局任务状态。
 * @param[out] output 接收统计副本；参数错误时保持原值。
 * @return DAEMON_OK或参数错误。
 */
DaemonStatus_t DaemonGetStats(const DaemonInstance *instance, DaemonStats_t *output);

/** @brief 兼容旧调用：按HAL当前时刻报告一份合法通信和有效数据。
 * @param[in] instance 来源实例。
 * @note 新接收路径应使用DaemonReportAt并传入原接收时间和校验结果。
 */
void DaemonReload(DaemonInstance *instance);

/** @brief 检查通信是否仍在期限内，不表示关键数据有效。
 * @param[in] instance 来源实例。
 * @return 1为通信在线，0为未收到或已过期。
 */
uint8_t DaemonIsOnline(DaemonInstance *instance);

#endif
