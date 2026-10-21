/** @file bsp_log.h
 * @brief 声明固定事件目录、有界提交、只读历史和日志任务接口。
 * @note 先调用 BSPLogInit；提交只允许启动代码和普通任务，事件数据会在返回前复制。
 */
#ifndef MC02_BSP_LOG_H
#define MC02_BSP_LOG_H

#include <stddef.h>
#include <stdint.h>

#define BSP_LOG_QUEUE_CAPACITY 32u
#define BSP_LOG_NORMAL_QUEUE_LIMIT 24u
#define BSP_LOG_TEXT_MAX 48u
#define BSP_LOG_PAYLOAD_MAX 96u
#define BSP_LOG_HISTORY_CAPACITY 16u
#define BSP_LOG_OUTPUT_CAPACITY 256u
#define BSP_LOG_MAX_FIELDS 8u
#define BSP_LOG_SOURCE_CAPACITY 32u
#define BSP_LOG_WORKSPACE_MAX 16384u

#ifndef MC02_LOG_ORDINARY
#define MC02_LOG_ORDINARY 0
#endif

typedef enum
{
    LOG_SEVERITY_INFO = 0,
    LOG_SEVERITY_WARNING,
    LOG_SEVERITY_ERROR
} BSPLogSeverity_t;

typedef enum
{
    LOG_CATEGORY_FAULT = 0,
    LOG_CATEGORY_STATUS,
    LOG_CATEGORY_NORMAL
} BSPLogCategory_t;

typedef enum
{
    BSP_LOG_VALUE_U32 = 0,
    BSP_LOG_VALUE_I32,
    BSP_LOG_VALUE_F32,
    BSP_LOG_VALUE_TEXT
} BSPLogValueType_t;

/** @brief 提交、读取和处理接口的结果。 */
typedef enum
{
    BSP_LOG_STATUS_OK = 0,
    BSP_LOG_STATUS_QUEUED,
    BSP_LOG_STATUS_MERGED,
    BSP_LOG_STATUS_RETAINED,
    BSP_LOG_STATUS_FILTERED,
    BSP_LOG_STATUS_RATE_LIMITED,
    BSP_LOG_STATUS_FULL,
    BSP_LOG_STATUS_BAD_ARGUMENT,
    BSP_LOG_STATUS_NOT_INITIALIZED,
    BSP_LOG_STATUS_BAD_CONTEXT,
    BSP_LOG_STATUS_NOT_FOUND,
    BSP_LOG_STATUS_EMPTY,
    BSP_LOG_STATUS_PROCESSED,
    BSP_LOG_STATUS_TASK_CREATE_FAILED
} BSPLogStatus_t;

typedef enum
{
#define BSP_LOG_SOURCE(name, value, module, path) name = value,
#include "bsp_log_sources.def"
#undef BSP_LOG_SOURCE
    BSP_LOG_SOURCE_COUNT = 32
} BSPLogSourceId_t;

typedef enum
{
#define BSP_LOG_EVENT(name, value, source, severity, category, file, message, types, labels, units) name = value,
#include "bsp_log_events.def"
#undef BSP_LOG_EVENT
} BSPLogEventId_t;

/** @brief 一次提交使用的字段视图；文本指针只在 BSPLogSubmit() 内读取。 */
typedef struct
{
    const char *data;
    size_t length;
} BSPLogTextView_t;

/** @brief 定长数值或有长度的文本字段。 */
typedef struct
{
    BSPLogValueType_t type;
    union
    {
        uint32_t u32;
        int32_t i32;
        float f32;
        BSPLogTextView_t text;
    } value;
} BSPLogField_t;

/** @brief 供只读查询复制的事件记录；payload为内部类型-长度-数据格式。 */
typedef struct
{
    uint32_t sequence;
    uint32_t first_ms;
    uint32_t last_ms;
    uint32_t repeat_count;
    BSPLogEventId_t event_id;
    uint8_t category;
    uint8_t severity;
    uint8_t flags;
    uint8_t payload_length;
    uint8_t payload[BSP_LOG_PAYLOAD_MAX];
} BSPLogHistoryRecord_t;

/** @brief 日志队列、历史、输出和任务的只读运行状态。 */
typedef struct
{
    uint32_t submitted;
    uint32_t queued;
    uint32_t merged;
    uint32_t filtered;
    uint32_t rate_limited;
    uint32_t queue_full;
    uint32_t evicted_normal;
    uint32_t retained_only;
    uint32_t shortened_text_fields;
    uint32_t format_failures;
    uint32_t output_failures;
    uint32_t task_create_failures;
    uint32_t process_cycles;
    uint32_t first_fault_sequence;
    uint16_t queue_depth;
    uint16_t normal_queue_depth;
    uint16_t queue_high_water;
    uint8_t history_count;
    uint8_t backend_available;
    uint8_t task_started;
    uint8_t task_create_pending;
    uint8_t ordinary_enabled;
    uint8_t processing;
} BSPLogState_t;

/** @brief 以整数形式保存的32位字段。 */
#define BSP_LOG_FIELD_U32(value_) ((BSPLogField_t){.type = BSP_LOG_VALUE_U32, .value.u32 = (uint32_t)(value_)})
/** @brief 以有符号整数形式保存的32位字段。 */
#define BSP_LOG_FIELD_I32(value_) ((BSPLogField_t){.type = BSP_LOG_VALUE_I32, .value.i32 = (int32_t)(value_)})
/** @brief 以IEEE 754单精度保存的字段；NaN和Inf由处理端显示标记。 */
#define BSP_LOG_FIELD_F32(value_) ((BSPLogField_t){.type = BSP_LOG_VALUE_F32, .value.f32 = (float)(value_)})
/** @brief 复制指定长度的文本字段；提交后可以复用或释放原缓冲。 */
#define BSP_LOG_FIELD_TEXT(data_, length_) ((BSPLogField_t){.type = BSP_LOG_VALUE_TEXT, .value.text = {.data = (data_), .length = (length_)}})
/** @brief 有界读取零结尾文字；来源须可读至结尾，或在limit字节无结尾时额外可读1字节。 */
#define BSP_LOG_FIELD_CSTR_LIMIT(data_, limit_) BSPLogFieldFromCString((data_), (limit_))
/** @brief 为已知字符串字面量生成有明确长度的文本字段。 */
#define BSP_LOG_FIELD_TEXT_LITERAL(text_) BSP_LOG_FIELD_TEXT((text_), sizeof(text_) - 1u)
/** @brief 十六进制显示的32位字段。 */
#define BSP_LOG_FIELD_HEX32(value_) BSP_LOG_FIELD_U32(value_)

/** @brief 初始化队列、历史和RTT后端；重复调用保留本次启动的记录。
 * @note 应在DWT初始化后、看门狗及其诊断之前调用；不创建日志任务。
 */
void BSPLogInit(void);

/** @brief 创建静态日志任务。
 * @return 成功返回BSP_LOG_STATUS_OK；创建失败返回TASK_CREATE_FAILED，可在调度器启动前重试。
 * @note 只在初始化后、调度器启动前从普通启动代码调用；任务创建失败不代表核心初始化失败。
 */
BSPLogStatus_t BSPLogTaskInit(void);

/** @brief 提交固定目录中的类型化事件，复制所有字段后立即返回。
 * @param[in] event_id 固定事件目录编号；未知编号返回BAD_ARGUMENT。
 * @param[in] fields 字段数组；字段类型和数量必须符合目录，文本指针须在本次调用期间可读。
 * @param[in] field_count 字段数量，范围0～BSP_LOG_MAX_FIELDS。
 * @return QUEUED、MERGED、RETAINED或具体过滤、限速、容量、参数、初始化及上下文结果。
 * @note 只允许启动代码和普通任务。文本字段最多保留48字节并累计缩短次数；单条编码数据最多96字节。失败不保存调用者指针。
 */
BSPLogStatus_t BSPLogSubmit(BSPLogEventId_t event_id, const BSPLogField_t *fields, uint8_t field_count);

/** @brief 处理最多指定数量的待发记录。
 * @param[in] maximum_records 正常任务为1～2；启动失败循环传1。调度器运行后只允许日志任务调用。
 * @return 本轮处理结果；空队列返回EMPTY，越界、未初始化或调用上下文错误有独立状态。
 * @note 格式化和RTT写入在临界区外执行。每条记录只尝试写一次；调度器未启动时最多处理一条。
 */
BSPLogStatus_t BSPLogProcessPending(uint8_t maximum_records);

/** @brief 复制日志状态；查询不推进队列或刷新事件时间。
 * @param[out] output 接收副本；NULL或未初始化时保持原内容。
 * @return 成功返回OK，否则返回参数或初始化错误。
 */
BSPLogStatus_t BSPLogGetState(BSPLogState_t *output);

/** @brief 按非零事件序号复制首次故障或最近16条关键记录。
 * @param[in] sequence 事件序号；序号被覆盖后返回NOT_FOUND。
 * @param[out] output 接收副本；失败时保持原内容。
 * @return 成功返回OK，否则返回参数、初始化或未找到。
 */
BSPLogStatus_t BSPLogGetHistory(uint32_t sequence, BSPLogHistoryRecord_t *output);

/** @brief 统计零结尾文本的可读前缀；limit为上限，字符串未在上限内结束时会多读1字节。 */
BSPLogField_t BSPLogFieldFromCString(const char *text, size_t limit);

/* 每个事件都提供固定名称的提交封装，目录校验字段类型和顺序。 */
#define BSP_LOG_EVENT(name, value, source, severity, category, file, message, types, labels, units) \
    static inline BSPLogStatus_t BSPLogSubmit_##name(const BSPLogField_t *fields, uint8_t count) \
    { return BSPLogSubmit((name), fields, count); }
#include "bsp_log_events.def"
#undef BSP_LOG_EVENT

/* 字段封装宏不接受格式串，生产者只提交原始值。 */
#if DISABLE_LOG_SYSTEM
#define BSP_LOG_EVENT_0(name) do { if (0) (void)BSPLogSubmit_##name(NULL, 0u); } while (0)
#define BSP_LOG_EVENT_1(name, a) do { if (0) { const BSPLogField_t _f[] = {a}; (void)BSPLogSubmit_##name(_f, 1u); } } while (0)
#define BSP_LOG_EVENT_2(name, a, b) do { if (0) { const BSPLogField_t _f[] = {a, b}; (void)BSPLogSubmit_##name(_f, 2u); } } while (0)
#define BSP_LOG_EVENT_3(name, a, b, c) do { if (0) { const BSPLogField_t _f[] = {a, b, c}; (void)BSPLogSubmit_##name(_f, 3u); } } while (0)
#define BSP_LOG_EVENT_4(name, a, b, c, d) do { if (0) { const BSPLogField_t _f[] = {a, b, c, d}; (void)BSPLogSubmit_##name(_f, 4u); } } while (0)
#define BSP_LOG_EVENT_5(name, a, b, c, d, e) do { if (0) { const BSPLogField_t _f[] = {a, b, c, d, e}; (void)BSPLogSubmit_##name(_f, 5u); } } while (0)
#define BSP_LOG_EVENT_6(name, a, b, c, d, e, f) do { if (0) { const BSPLogField_t _f[] = {a, b, c, d, e, f}; (void)BSPLogSubmit_##name(_f, 6u); } } while (0)
#define BSP_LOG_EVENT_7(name, a, b, c, d, e, f, g) do { if (0) { const BSPLogField_t _f[] = {a, b, c, d, e, f, g}; (void)BSPLogSubmit_##name(_f, 7u); } } while (0)
#define BSP_LOG_EVENT_8(name, a, b, c, d, e, f, g, h) do { if (0) { const BSPLogField_t _f[] = {a, b, c, d, e, f, g, h}; (void)BSPLogSubmit_##name(_f, 8u); } } while (0)
#else
#define BSP_LOG_EVENT_0(name) do { (void)BSPLogSubmit_##name(NULL, 0u); } while (0)
#define BSP_LOG_EVENT_1(name, a) do { const BSPLogField_t _f[] = {a}; (void)BSPLogSubmit_##name(_f, 1u); } while (0)
#define BSP_LOG_EVENT_2(name, a, b) do { const BSPLogField_t _f[] = {a, b}; (void)BSPLogSubmit_##name(_f, 2u); } while (0)
#define BSP_LOG_EVENT_3(name, a, b, c) do { const BSPLogField_t _f[] = {a, b, c}; (void)BSPLogSubmit_##name(_f, 3u); } while (0)
#define BSP_LOG_EVENT_4(name, a, b, c, d) do { const BSPLogField_t _f[] = {a, b, c, d}; (void)BSPLogSubmit_##name(_f, 4u); } while (0)
#define BSP_LOG_EVENT_5(name, a, b, c, d, e) do { const BSPLogField_t _f[] = {a, b, c, d, e}; (void)BSPLogSubmit_##name(_f, 5u); } while (0)
#define BSP_LOG_EVENT_6(name, a, b, c, d, e, f) do { const BSPLogField_t _f[] = {a, b, c, d, e, f}; (void)BSPLogSubmit_##name(_f, 6u); } while (0)
#define BSP_LOG_EVENT_7(name, a, b, c, d, e, f, g) do { const BSPLogField_t _f[] = {a, b, c, d, e, f, g}; (void)BSPLogSubmit_##name(_f, 7u); } while (0)
#define BSP_LOG_EVENT_8(name, a, b, c, d, e, f, g, h) do { const BSPLogField_t _f[] = {a, b, c, d, e, f, g, h}; (void)BSPLogSubmit_##name(_f, 8u); } while (0)
#endif

#ifndef LOG_TEST
#define LOG_TEST 0
#endif

#if LOG_TEST
/** @brief 向固定事件目录提交三条合法类型化自测记录；普通事件需用诊断构建选项允许。 */
void LogSelfTest(void);
#endif

#endif /* MC02_BSP_LOG_H */
