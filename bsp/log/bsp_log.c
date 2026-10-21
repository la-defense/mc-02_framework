/** @file bsp_log.c
 * @brief 固定类型事件队列、关键记录历史、限速、合并和无格式串渲染。
 * @note 队列和历史放在日志专用DTCM段；BSPLogInit()首次调用时显式清零。
 */
#include "bsp_log.h"
#include "bsp_log_port.h"

#include <math.h>
#include <string.h>

#define BSP_LOG_TEXT_SHORTENED 0x01u
#define BSP_LOG_FIELD_HEADER_SIZE 2u
#define BSP_LOG_TOKEN_INTERVAL_MS 50u
#define BSP_LOG_TOKEN_BURST 8u
#define BSP_LOG_RATE_PER_SECOND 20u

typedef struct
{
    BSPLogEventId_t id;
    BSPLogSourceId_t source;
    BSPLogSeverity_t severity;
    BSPLogCategory_t category;
    const char *file;
    const char *message;
    const char *field_types;
    const char *field_labels;
    const char *field_units;
} BSPLogEventSpec_t;

typedef struct
{
    uint8_t used;
    uint8_t source_id;
    BSPLogHistoryRecord_t record;
} BSPLogQueueSlot_t;

typedef struct
{
    uint8_t valid;
    uint8_t source_id;
    BSPLogHistoryRecord_t record;
} BSPLogMergeSlot_t;

typedef struct
{
    BSPLogQueueSlot_t queue[BSP_LOG_QUEUE_CAPACITY];
    BSPLogHistoryRecord_t history[BSP_LOG_HISTORY_CAPACITY];
    uint8_t history_valid[BSP_LOG_HISTORY_CAPACITY];
    BSPLogHistoryRecord_t first_fault;
    uint8_t first_fault_valid;
    BSPLogMergeSlot_t last_critical[BSP_LOG_SOURCE_CAPACITY];
    BSPLogHistoryRecord_t processing_record;
    char output[BSP_LOG_OUTPUT_CAPACITY];
    uint32_t next_sequence;
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
    uint32_t token_tick;
    uint16_t queue_depth;
    uint16_t normal_queue_depth;
    uint16_t queue_high_water;
    uint8_t history_next;
    uint8_t history_count;
    uint8_t tokens;
    uint8_t backend_available;
    uint8_t task_started;
    uint8_t task_create_pending;
    uint8_t processing;
} BSPLogWorkspace_t;

typedef struct
{
    char *data;
    size_t length;
    uint8_t overflowed;
} BSPLogTextBuilder_t;

#if defined(__GNUC__)
#define BSP_LOG_DTCM __attribute__((section(".log_workspace"), aligned(32)))
#else
#define BSP_LOG_DTCM
#endif

/* 这块NOLOAD内存不由启动代码清零，初始化入口负责第一次完整清理。 */
static BSPLogWorkspace_t log_workspace BSP_LOG_DTCM;
static uint8_t log_workspace_initialized;

#define BSP_LOG_EVENT(name, value, source, severity, category, file, message, types, labels, units) \
    {name, source, severity, category, file, message, types, labels, units},
static const BSPLogEventSpec_t log_events[] = {
#include "bsp_log_events.def"
};
#undef BSP_LOG_EVENT

static uint32_t BSPLogSaturatingAdd(uint32_t value, uint32_t add)
{
    return (UINT32_MAX - value < add) ? UINT32_MAX : value + add;
}

static void BSPLogCount(uint32_t *counter)
{
    *counter = BSPLogSaturatingAdd(*counter, 1u);
}

static const BSPLogEventSpec_t *BSPLogFindEvent(BSPLogEventId_t id)
{
    /* 目录按源文件收录，编号不保证连续排序；逐项匹配避免误查另一条事件。 */
    for (size_t i = 0u; i < sizeof(log_events) / sizeof(log_events[0]); ++i)
    {
        if (log_events[i].id == id)
            return &log_events[i];
    }
    return NULL;
}

static uint8_t BSPLogSequenceBefore(uint32_t left, uint32_t right)
{
    return ((int32_t)(left - right) < 0) ? 1u : 0u;
}

static uint32_t BSPLogNextSequence(void)
{
    uint32_t value = log_workspace.next_sequence;
    if (value == 0u)
        value = 1u;
    log_workspace.next_sequence = value + 1u;
    if (log_workspace.next_sequence == 0u)
        log_workspace.next_sequence = 1u;
    return value;
}

static uint8_t BSPLogPayloadEqual(const BSPLogHistoryRecord_t *left,
                                 const BSPLogHistoryRecord_t *right)
{
    return left->event_id == right->event_id &&
           left->payload_length == right->payload_length &&
           memcmp(left->payload, right->payload, left->payload_length) == 0;
}

static int BSPLogFindQueueSequence(uint32_t sequence)
{
    for (uint32_t i = 0u; i < BSP_LOG_QUEUE_CAPACITY; ++i)
    {
        if (log_workspace.queue[i].used &&
            log_workspace.queue[i].record.sequence == sequence)
            return (int)i;
    }
    return -1;
}

static int BSPLogFindHistorySequence(uint32_t sequence)
{
    for (uint32_t i = 0u; i < BSP_LOG_HISTORY_CAPACITY; ++i)
    {
        if (log_workspace.history_valid[i] &&
            log_workspace.history[i].sequence == sequence)
            return (int)i;
    }
    return -1;
}

static void BSPLogStoreCriticalHistory(const BSPLogHistoryRecord_t *record)
{
    uint8_t index = log_workspace.history_next;
    log_workspace.history[index] = *record;
    log_workspace.history_valid[index] = 1u;
    log_workspace.history_next = (uint8_t)((index + 1u) % BSP_LOG_HISTORY_CAPACITY);
    if (log_workspace.history_count < BSP_LOG_HISTORY_CAPACITY)
        log_workspace.history_count++;

    if (record->category == LOG_CATEGORY_FAULT && !log_workspace.first_fault_valid)
    {
        /* 首次关键故障单独保存，最近记录循环覆盖不会删除它。 */
        log_workspace.first_fault = *record;
        log_workspace.first_fault_valid = 1u;
    }
}

static uint8_t BSPLogHasNormalQueueRoom(void)
{
    return log_workspace.queue_depth < BSP_LOG_QUEUE_CAPACITY &&
           log_workspace.normal_queue_depth < BSP_LOG_NORMAL_QUEUE_LIMIT;
}

static uint8_t BSPLogRefillTokens(uint32_t now)
{
    uint32_t elapsed = now - log_workspace.token_tick;
    uint32_t refill = elapsed / BSP_LOG_TOKEN_INTERVAL_MS;
    if (refill == 0u)
        return log_workspace.tokens;

    log_workspace.token_tick += refill * BSP_LOG_TOKEN_INTERVAL_MS;
    uint32_t available = (uint32_t)log_workspace.tokens + refill;
    log_workspace.tokens = (uint8_t)(available > BSP_LOG_TOKEN_BURST ? BSP_LOG_TOKEN_BURST : available);
    return log_workspace.tokens;
}

static int BSPLogFindOldest(uint8_t normal_only)
{
    int found = -1;
    for (uint32_t i = 0u; i < BSP_LOG_QUEUE_CAPACITY; ++i)
    {
        BSPLogQueueSlot_t *slot = &log_workspace.queue[i];
        if (!slot->used)
            continue;
        uint8_t is_normal = slot->record.category == LOG_CATEGORY_NORMAL;
        if ((normal_only && !is_normal) || (!normal_only && is_normal))
            continue;
        if (found < 0 || BSPLogSequenceBefore(slot->record.sequence,
                                               log_workspace.queue[(uint32_t)found].record.sequence))
            found = (int)i;
    }
    return found;
}

static int BSPLogFindFreeQueueSlot(void)
{
    for (uint32_t i = 0u; i < BSP_LOG_QUEUE_CAPACITY; ++i)
    {
        if (!log_workspace.queue[i].used)
            return (int)i;
    }
    return -1;
}

static void BSPLogRemoveQueueSlot(uint32_t index)
{
    if (log_workspace.queue[index].record.category == LOG_CATEGORY_NORMAL &&
        log_workspace.normal_queue_depth > 0u)
        log_workspace.normal_queue_depth--;
    if (log_workspace.queue_depth > 0u)
        log_workspace.queue_depth--;
    log_workspace.queue[index].used = 0u;
}

static void BSPLogQueueRecord(const BSPLogHistoryRecord_t *record, uint8_t source_id)
{
    int slot_index = BSPLogFindFreeQueueSlot();
    if (slot_index < 0)
        return;
    BSPLogQueueSlot_t *slot = &log_workspace.queue[(uint32_t)slot_index];
    slot->record = *record;
    slot->source_id = source_id;
    slot->used = 1u;
    log_workspace.queue_depth++;
    if (record->category == LOG_CATEGORY_NORMAL)
        log_workspace.normal_queue_depth++;
    if (log_workspace.queue_depth > log_workspace.queue_high_water)
        log_workspace.queue_high_water = log_workspace.queue_depth;
    BSPLogCount(&log_workspace.queued);
}

static uint8_t BSPLogEncodeFields(const BSPLogEventSpec_t *spec,
                                  const BSPLogField_t *fields,
                                  uint8_t field_count,
                                  BSPLogHistoryRecord_t *record,
                                  uint32_t *shortened_count)
{
    size_t expected = strlen(spec->field_types);
    size_t output = 0u;
    *shortened_count = 0u;
    if (expected != field_count || field_count > BSP_LOG_MAX_FIELDS ||
        (field_count != 0u && fields == NULL))
        return 0u;

    for (uint8_t i = 0u; i < field_count; ++i)
    {
        char schema_type = spec->field_types[i];
        const BSPLogField_t *field = &fields[i];
        uint8_t encoded_type;
        const uint8_t *raw = NULL;
        uint8_t raw_length = 4u;
        uint32_t integer_bits = 0u;
        uint8_t float_bits[4];

        if (schema_type == 'T')
        {
            if (field->type != BSP_LOG_VALUE_TEXT ||
                (field->value.text.length != 0u && field->value.text.data == NULL))
                return 0u;
            encoded_type = 'T';
            size_t reserved = 0u;
            for (uint8_t later = (uint8_t)(i + 1u); later < field_count; ++later)
                reserved += spec->field_types[later] == 'T' ? BSP_LOG_FIELD_HEADER_SIZE
                                                             : BSP_LOG_FIELD_HEADER_SIZE + sizeof(uint32_t);
            size_t available = BSP_LOG_PAYLOAD_MAX - output - BSP_LOG_FIELD_HEADER_SIZE - reserved;
            size_t copy_length = field->value.text.length;
            if (copy_length > BSP_LOG_TEXT_MAX)
                copy_length = BSP_LOG_TEXT_MAX;
            if (copy_length > available)
                copy_length = available;
            raw_length = (uint8_t)copy_length;
            raw = (const uint8_t *)field->value.text.data;
            if (field->value.text.length > copy_length)
            {
                record->flags |= BSP_LOG_TEXT_SHORTENED;
                (*shortened_count)++;
            }
        }
        else if (schema_type == 'F')
        {
            if (field->type != BSP_LOG_VALUE_F32)
                return 0u;
            encoded_type = 'F';
            memcpy(float_bits, &field->value.f32, sizeof(float_bits));
            raw = float_bits;
        }
        else
        {
            if (field->type != (schema_type == 'I' ? BSP_LOG_VALUE_I32 : BSP_LOG_VALUE_U32))
                return 0u;
            encoded_type = (uint8_t)schema_type;
            integer_bits = (schema_type == 'I') ? (uint32_t)field->value.i32 : field->value.u32;
            raw = (const uint8_t *)&integer_bits;
        }

        if (output + BSP_LOG_FIELD_HEADER_SIZE + raw_length > BSP_LOG_PAYLOAD_MAX)
            return 0u;
        record->payload[output++] = encoded_type;
        record->payload[output++] = raw_length;
        for (uint8_t j = 0u; j < raw_length; ++j)
            record->payload[output++] = raw[j];
    }
    record->payload_length = (uint8_t)output;
    return 1u;
}

static void BSPLogAppendByte(BSPLogTextBuilder_t *builder, char value)
{
    if (builder->length + 1u < BSP_LOG_OUTPUT_CAPACITY)
        builder->data[builder->length++] = value;
    else
        builder->overflowed = 1u;
}

static void BSPLogAppendString(BSPLogTextBuilder_t *builder, const char *text)
{
    if (text == NULL)
        return;
    while (*text != '\0')
        BSPLogAppendByte(builder, *text++);
}

static void BSPLogAppendUnsigned(BSPLogTextBuilder_t *builder, uint32_t value, uint8_t base)
{
    char digits[10];
    uint8_t used = 0u;
    const char *alphabet = "0123456789ABCDEF";
    do
    {
        digits[used++] = alphabet[value % base];
        value /= base;
    } while (value != 0u && used < sizeof(digits));
    while (used != 0u)
        BSPLogAppendByte(builder, digits[--used]);
}

static void BSPLogAppendSigned(BSPLogTextBuilder_t *builder, int32_t value)
{
    uint32_t magnitude;
    if (value < 0)
    {
        BSPLogAppendByte(builder, '-');
        magnitude = (uint32_t)(-(int64_t)value);
    }
    else
        magnitude = (uint32_t)value;
    BSPLogAppendUnsigned(builder, magnitude, 10u);
}

static void BSPLogAppendHex32(BSPLogTextBuilder_t *builder, uint32_t value)
{
    BSPLogAppendString(builder, "0x");
    for (int8_t shift = 28; shift >= 0; shift -= 4)
        BSPLogAppendByte(builder, "0123456789ABCDEF"[(value >> (uint8_t)shift) & 0x0Fu]);
}

static void BSPLogAppendFloat(BSPLogTextBuilder_t *builder, float value)
{
    union
    {
        float f;
        uint32_t u;
    } bits;
    bits.f = value;

    if (isnan(value))
    {
        BSPLogAppendString(builder, "NaN");
        return;
    }
    if (isinf(value))
    {
        BSPLogAppendString(builder, signbit(value) ? "-Inf" : "Inf");
        return;
    }
    if (fabsf(value) > 4294967.0f)
    {
        BSPLogAppendString(builder, "raw=");
        BSPLogAppendHex32(builder, bits.u);
        return;
    }

    double magnitude = fabs((double)value) * 1000.0;
    uint32_t milli = (uint32_t)(magnitude + 0.5);
    if (signbit(value))
        BSPLogAppendByte(builder, '-');
    BSPLogAppendUnsigned(builder, milli / 1000u, 10u);
    BSPLogAppendByte(builder, '.');
    uint32_t fraction = milli % 1000u;
    BSPLogAppendByte(builder, (char)('0' + fraction / 100u));
    BSPLogAppendByte(builder, (char)('0' + (fraction / 10u) % 10u));
    BSPLogAppendByte(builder, (char)('0' + fraction % 10u));
}

static void BSPLogAppendCatalogPart(BSPLogTextBuilder_t *builder, const char *packed, uint8_t index)
{
    if (packed == NULL)
        return;
    uint8_t current = 0u;
    while (current < index && *packed != '\0')
    {
        if (*packed++ == '|')
            current++;
    }
    while (*packed != '\0' && *packed != '|')
        BSPLogAppendByte(builder, *packed++);
}

static uint8_t BSPLogRender(const BSPLogEventSpec_t *spec,
                            const BSPLogHistoryRecord_t *record,
                            char output[BSP_LOG_OUTPUT_CAPACITY],
                            size_t *output_length)
{
    BSPLogTextBuilder_t builder = {.data = output, .length = 0u};
    char severity = spec->severity == LOG_SEVERITY_ERROR ? 'E' :
                    spec->severity == LOG_SEVERITY_WARNING ? 'W' : 'I';
    BSPLogAppendByte(&builder, '[');
    BSPLogAppendUnsigned(&builder, record->sequence, 10u);
    BSPLogAppendByte(&builder, '@');
    BSPLogAppendUnsigned(&builder, record->first_ms, 10u);
    BSPLogAppendString(&builder, "ms][");
    BSPLogAppendByte(&builder, severity);
    BSPLogAppendString(&builder, "][");
    BSPLogAppendString(&builder, spec->file);
    BSPLogAppendString(&builder, "] ");
    BSPLogAppendString(&builder, spec->message);

    size_t offset = 0u;
    uint8_t field_index = 0u;
    while (spec->field_types[field_index] != '\0')
    {
        if (offset + BSP_LOG_FIELD_HEADER_SIZE > record->payload_length)
            return 0u;
        uint8_t type = record->payload[offset++];
        uint8_t length = record->payload[offset++];
        if (offset + length > record->payload_length || type != (uint8_t)spec->field_types[field_index])
            return 0u;

        BSPLogAppendByte(&builder, ' ');
        BSPLogAppendCatalogPart(&builder, spec->field_labels, field_index);
        BSPLogAppendByte(&builder, '=');
        if (type == 'T')
        {
            for (uint8_t i = 0u; i < length; ++i)
            {
                char c = (char)record->payload[offset + i];
                BSPLogAppendByte(&builder, ((uint8_t)c < 0x20u || (uint8_t)c == 0x7Fu) ? '?' : c);
            }
        }
        else if (type == 'F')
        {
            if (length != sizeof(float))
                return 0u;
            float value;
            memcpy(&value, &record->payload[offset], sizeof(value));
            BSPLogAppendFloat(&builder, value);
        }
        else
        {
            if (length != sizeof(uint32_t))
                return 0u;
            uint32_t value;
            memcpy(&value, &record->payload[offset], sizeof(value));
            if (type == 'I')
                BSPLogAppendSigned(&builder, (int32_t)value);
            else if (type == 'H')
                BSPLogAppendHex32(&builder, value);
            else
                BSPLogAppendUnsigned(&builder, value, 10u);
        }
        BSPLogAppendCatalogPart(&builder, spec->field_units, field_index);
        offset += length;
        field_index++;
    }

    if (offset != record->payload_length)
        return 0u;
    if (record->repeat_count > 1u)
    {
        BSPLogAppendString(&builder, " repeat=");
        BSPLogAppendUnsigned(&builder, record->repeat_count, 10u);
    }
    if ((record->flags & BSP_LOG_TEXT_SHORTENED) != 0u)
        BSPLogAppendString(&builder, " text-shortened");
    BSPLogAppendByte(&builder, '\n');
    if (builder.overflowed)
        return 0u;
    output[builder.length] = '\0';
    *output_length = builder.length;
    return 1u;
}

BSPLogField_t BSPLogFieldFromCString(const char *text, size_t limit)
{
    BSPLogField_t field = {.type = BSP_LOG_VALUE_TEXT, .value.text = {.data = text, .length = 0u}};
    if (text == NULL)
        return field;

    if (limit > BSP_LOG_TEXT_MAX)
        limit = BSP_LOG_TEXT_MAX;

    /* 最多读取limit+1字节，以便分辨恰好装满和需要缩短的文字。 */
    while (field.value.text.length <= limit && text[field.value.text.length] != '\0')
        field.value.text.length++;
    return field;
}

void BSPLogInit(void)
{
    if (log_workspace_initialized)
        return;

    memset(&log_workspace, 0, sizeof(log_workspace));
    log_workspace.next_sequence = 1u;
    log_workspace.tokens = BSP_LOG_TOKEN_BURST;
    log_workspace.token_tick = BSPLogPortNowMs();
    log_workspace.backend_available = BSPLogPortInit();
    log_workspace_initialized = 1u;
}

BSPLogStatus_t BSPLogTaskInit(void)
{
    if (!log_workspace_initialized)
        return BSP_LOG_STATUS_NOT_INITIALIZED;

    BSPLogPortEnterCritical();
    if (log_workspace.task_started)
    {
        BSPLogPortExitCritical();
        return BSP_LOG_STATUS_OK;
    }
    if (log_workspace.task_create_pending)
    {
        BSPLogPortExitCritical();
        return BSP_LOG_STATUS_BAD_CONTEXT;
    }
    log_workspace.task_create_pending = 1u;
    BSPLogPortExitCritical();

    uint8_t created = BSPLogPortStartWorker(BSPLogWorkerEntry);
    BSPLogPortEnterCritical();
    log_workspace.task_create_pending = 0u;
    if (created)
        log_workspace.task_started = 1u;
    else
        BSPLogCount(&log_workspace.task_create_failures);
    BSPLogPortExitCritical();
    return created ? BSP_LOG_STATUS_OK : BSP_LOG_STATUS_TASK_CREATE_FAILED;
}

BSPLogStatus_t BSPLogSubmit(BSPLogEventId_t event_id,
                            const BSPLogField_t *fields,
                            uint8_t field_count)
{
    if (!log_workspace_initialized)
        return BSP_LOG_STATUS_NOT_INITIALIZED;
    if (BSPLogPortInInterrupt())
        return BSP_LOG_STATUS_BAD_CONTEXT;

    const BSPLogEventSpec_t *spec = BSPLogFindEvent(event_id);
    if (spec == NULL)
        return BSP_LOG_STATUS_BAD_ARGUMENT;

    BSPLogHistoryRecord_t candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.event_id = event_id;
    candidate.category = (uint8_t)spec->category;
    candidate.severity = (uint8_t)spec->severity;
    uint32_t shortened = 0u;
    if (!BSPLogEncodeFields(spec, fields, field_count, &candidate, &shortened))
        return BSP_LOG_STATUS_BAD_ARGUMENT;
    uint32_t now = BSPLogPortNowMs();

    BSPLogPortEnterCritical();
    BSPLogCount(&log_workspace.submitted);
    if (spec->category == LOG_CATEGORY_NORMAL && !MC02_LOG_ORDINARY)
    {
        BSPLogCount(&log_workspace.filtered);
        BSPLogPortExitCritical();
        return BSP_LOG_STATUS_FILTERED;
    }
    if (spec->category == LOG_CATEGORY_NORMAL)
    {
        if (BSPLogRefillTokens(now) == 0u)
        {
            BSPLogCount(&log_workspace.rate_limited);
            BSPLogPortExitCritical();
            return BSP_LOG_STATUS_RATE_LIMITED;
        }
        if (!BSPLogHasNormalQueueRoom())
        {
            BSPLogCount(&log_workspace.queue_full);
            BSPLogPortExitCritical();
            return BSP_LOG_STATUS_FULL;
        }
    }

    if (shortened != 0u)
        log_workspace.shortened_text_fields = BSPLogSaturatingAdd(log_workspace.shortened_text_fields, shortened);

    uint8_t source_id = (uint8_t)spec->source;
    if (spec->category != LOG_CATEGORY_NORMAL && source_id < BSP_LOG_SOURCE_CAPACITY)
    {
        BSPLogMergeSlot_t *merge = &log_workspace.last_critical[source_id];
        if (merge->valid && merge->source_id == source_id &&
            BSPLogPayloadEqual(&merge->record, &candidate))
        {
            uint32_t repeats = BSPLogSaturatingAdd(merge->record.repeat_count, 1u);
            merge->record.repeat_count = repeats;
            merge->record.last_ms = now;
            int history = BSPLogFindHistorySequence(merge->record.sequence);
            if (history >= 0)
                log_workspace.history[(uint32_t)history] = merge->record;
            if (log_workspace.first_fault_valid &&
                log_workspace.first_fault.sequence == merge->record.sequence)
                log_workspace.first_fault = merge->record;
            int queued = BSPLogFindQueueSequence(merge->record.sequence);
            if (queued >= 0)
                log_workspace.queue[(uint32_t)queued].record = merge->record;
            BSPLogCount(&log_workspace.merged);
            BSPLogPortExitCritical();
            return BSP_LOG_STATUS_MERGED;
        }
    }

    candidate.sequence = BSPLogNextSequence();
    candidate.first_ms = now;
    candidate.last_ms = now;
    candidate.repeat_count = 1u;

    if (spec->category != LOG_CATEGORY_NORMAL)
    {
        BSPLogStoreCriticalHistory(&candidate);
        if (source_id < BSP_LOG_SOURCE_CAPACITY)
        {
            BSPLogMergeSlot_t *merge = &log_workspace.last_critical[source_id];
            merge->valid = 1u;
            merge->source_id = source_id;
            merge->record = candidate;
        }
    }

    if (spec->category == LOG_CATEGORY_NORMAL)
    {
        log_workspace.tokens--;
        int free_slot = BSPLogFindFreeQueueSlot();
        if (free_slot < 0)
        {
            BSPLogCount(&log_workspace.queue_full);
            BSPLogPortExitCritical();
            return BSP_LOG_STATUS_FULL;
        }
        BSPLogQueueRecord(&candidate, source_id);
        BSPLogPortExitCritical();
        return BSP_LOG_STATUS_QUEUED;
    }

    if (log_workspace.queue_depth >= BSP_LOG_QUEUE_CAPACITY)
    {
        int oldest_normal = BSPLogFindOldest(1u);
        if (oldest_normal >= 0)
        {
            BSPLogRemoveQueueSlot((uint32_t)oldest_normal);
            BSPLogCount(&log_workspace.evicted_normal);
        }
        else
        {
            BSPLogCount(&log_workspace.queue_full);
            BSPLogCount(&log_workspace.retained_only);
            BSPLogPortExitCritical();
            return BSP_LOG_STATUS_RETAINED;
        }
    }

    BSPLogQueueRecord(&candidate, source_id);
    BSPLogPortExitCritical();
    return BSP_LOG_STATUS_QUEUED;
}

static int BSPLogTakeRecord(uint8_t normal_only, BSPLogHistoryRecord_t *record)
{
    int slot_index = BSPLogFindOldest(normal_only);
    if (slot_index < 0)
        return 0;
    *record = log_workspace.queue[(uint32_t)slot_index].record;
    BSPLogRemoveQueueSlot((uint32_t)slot_index);
    return 1;
}

BSPLogStatus_t BSPLogProcessPending(uint8_t maximum_records)
{
    if (!log_workspace_initialized)
        return BSP_LOG_STATUS_NOT_INITIALIZED;
    if (maximum_records == 0u || maximum_records > 2u)
        return BSP_LOG_STATUS_BAD_ARGUMENT;
    if (BSPLogPortInInterrupt())
        return BSP_LOG_STATUS_BAD_CONTEXT;

    uint8_t scheduler_running = BSPLogPortSchedulerRunning();
    if ((scheduler_running && !BSPLogPortIsWorker()) || (!scheduler_running && maximum_records > 1u))
        return BSP_LOG_STATUS_BAD_CONTEXT;

    BSPLogPortEnterCritical();
    if (log_workspace.processing)
    {
        BSPLogPortExitCritical();
        return BSP_LOG_STATUS_BAD_CONTEXT;
    }
    log_workspace.processing = 1u;
    BSPLogCount(&log_workspace.process_cycles);
    BSPLogPortExitCritical();

    uint8_t processed = 0u;
    while (processed < maximum_records)
    {
        BSPLogPortEnterCritical();
        uint8_t take_normal = (maximum_records > 1u && processed == 1u) ? 1u : 0u;
        int found = BSPLogTakeRecord(take_normal, &log_workspace.processing_record);
        if (!found && take_normal)
            found = BSPLogTakeRecord(0u, &log_workspace.processing_record);
        else if (!found)
            found = BSPLogTakeRecord(1u, &log_workspace.processing_record);
        BSPLogHistoryRecord_t record = log_workspace.processing_record;
        BSPLogPortExitCritical();
        if (!found)
            break;
        processed++;

        const BSPLogEventSpec_t *spec = BSPLogFindEvent(record.event_id);
        size_t output_length = 0u;
        if (spec == NULL || !BSPLogRender(spec, &record, log_workspace.output, &output_length))
        {
            BSPLogPortEnterCritical();
            BSPLogCount(&log_workspace.format_failures);
            BSPLogPortExitCritical();
            continue;
        }

        uint8_t written = log_workspace.backend_available
                              ? BSPLogPortWriteWholeRecord(log_workspace.output, output_length)
                              : 0u;
        if (!written)
        {
            BSPLogPortEnterCritical();
            BSPLogCount(&log_workspace.output_failures);
            BSPLogPortExitCritical();
        }
    }

    BSPLogPortEnterCritical();
    log_workspace.processing = 0u;
    BSPLogPortExitCritical();
    return processed != 0u ? BSP_LOG_STATUS_PROCESSED : BSP_LOG_STATUS_EMPTY;
}

BSPLogStatus_t BSPLogGetState(BSPLogState_t *output)
{
    if (output == NULL)
        return BSP_LOG_STATUS_BAD_ARGUMENT;
    if (!log_workspace_initialized)
        return BSP_LOG_STATUS_NOT_INITIALIZED;

    BSPLogState_t candidate;
    BSPLogPortEnterCritical();
    memset(&candidate, 0, sizeof(candidate));
    candidate.submitted = log_workspace.submitted;
    candidate.queued = log_workspace.queued;
    candidate.merged = log_workspace.merged;
    candidate.filtered = log_workspace.filtered;
    candidate.rate_limited = log_workspace.rate_limited;
    candidate.queue_full = log_workspace.queue_full;
    candidate.evicted_normal = log_workspace.evicted_normal;
    candidate.retained_only = log_workspace.retained_only;
    candidate.shortened_text_fields = log_workspace.shortened_text_fields;
    candidate.format_failures = log_workspace.format_failures;
    candidate.output_failures = log_workspace.output_failures;
    candidate.task_create_failures = log_workspace.task_create_failures;
    candidate.process_cycles = log_workspace.process_cycles;
    candidate.first_fault_sequence = log_workspace.first_fault_valid ? log_workspace.first_fault.sequence : 0u;
    candidate.queue_depth = log_workspace.queue_depth;
    candidate.normal_queue_depth = log_workspace.normal_queue_depth;
    candidate.queue_high_water = log_workspace.queue_high_water;
    candidate.history_count = log_workspace.history_count;
    candidate.backend_available = log_workspace.backend_available;
    candidate.task_started = log_workspace.task_started;
    candidate.task_create_pending = log_workspace.task_create_pending;
    candidate.ordinary_enabled = MC02_LOG_ORDINARY ? 1u : 0u;
    candidate.processing = log_workspace.processing;
    BSPLogPortExitCritical();
    *output = candidate;
    return BSP_LOG_STATUS_OK;
}

BSPLogStatus_t BSPLogGetHistory(uint32_t sequence, BSPLogHistoryRecord_t *output)
{
    if (output == NULL || sequence == 0u)
        return BSP_LOG_STATUS_BAD_ARGUMENT;
    if (!log_workspace_initialized)
        return BSP_LOG_STATUS_NOT_INITIALIZED;

    BSPLogHistoryRecord_t candidate;
    uint8_t found = 0u;
    BSPLogPortEnterCritical();
    if (log_workspace.first_fault_valid && log_workspace.first_fault.sequence == sequence)
    {
        candidate = log_workspace.first_fault;
        found = 1u;
    }
    else
    {
        int index = BSPLogFindHistorySequence(sequence);
        if (index >= 0)
        {
            candidate = log_workspace.history[(uint32_t)index];
            found = 1u;
        }
    }
    BSPLogPortExitCritical();
    if (!found)
        return BSP_LOG_STATUS_NOT_FOUND;
    *output = candidate;
    return BSP_LOG_STATUS_OK;
}

void BSPLogWorkerEntry(void const *argument)
{
    (void)argument;
    for (;;)
    {
        (void)BSPLogProcessPending(2u);
        BSPLogPortDelayMs(5u);
    }
}

#if defined(BSP_LOG_TESTING)
/* 仅供主机边界用例验证跳过零的序号回绕，不进入固件构建。 */
void BSPLogTestSetNextSequence(uint32_t sequence)
{
    BSPLogPortEnterCritical();
    log_workspace.next_sequence = sequence;
    BSPLogPortExitCritical();
}
#endif

#if LOG_TEST
void LogSelfTest(void)
{
    static const char tail[] = "OK";
    BSP_LOG_EVENT_3(EV_BSP_LOG_BSP_LOG_C_01,
                    BSP_LOG_FIELD_TEXT_LITERAL(tail),
                    BSP_LOG_FIELD_U32(1234567u),
                    BSP_LOG_FIELD_I32(-42));
    BSP_LOG_EVENT_2(EV_BSP_LOG_BSP_LOG_C_02,
                    BSP_LOG_FIELD_U32(0xDEADBEEFu),
                    BSP_LOG_FIELD_HEX32(0xDEADBEEFu));
    BSP_LOG_EVENT_2(EV_BSP_LOG_BSP_LOG_C_03,
                    BSP_LOG_FIELD_F32(3.14159f),
                    BSP_LOG_FIELD_TEXT_LITERAL(tail));
}
#endif
