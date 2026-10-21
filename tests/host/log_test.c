/** @file log_test.c
 * @brief 直接验证生产类型化日志核心的字段复制、队列、关键历史和处理失败。
 * @note BSPLogPort_* 仅替换时钟、临界区、任务创建和RTT写入；测试检查在Release下有效。
 */
#include "bsp_log.h"
#include "bsp_log_port.h"
#include "test_check.h"

#include <stdarg.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(LOG_TEST_THREADS)
#include <pthread.h>
#endif

typedef struct
{
    BSPLogEventId_t id;
    BSPLogSourceId_t source;
    const char *types;
    const char *labels;
    const char *units;
    BSPLogCategory_t category;
} LogCatalogRow_t;

#define BSP_LOG_EVENT(name, value, source, severity, category, file, message, types, labels, units) \
    {name, source, types, labels, units, category},
static const LogCatalogRow_t catalog[] = {
#include "bsp_log_events.def"
};
#undef BSP_LOG_EVENT

#define LOG_OUTPUT_CAPACITY 1024u
static char output[LOG_OUTPUT_CAPACITY][BSP_LOG_OUTPUT_CAPACITY];
static size_t output_lengths[LOG_OUTPUT_CAPACITY];
static uint32_t output_count;
static uint32_t tick_ms;
static uint8_t fail_next_write;
static uint8_t worker_create_success = 1u;
static uint32_t worker_create_count;
static uint8_t interrupt_context;
static uint8_t scheduler_running;
static uint8_t worker_context;
static uint32_t format_call_count;

void BSPLogTestSetNextSequence(uint32_t sequence);

#if defined(LOG_TEST_THREADS)
static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t output_lock = PTHREAD_MUTEX_INITIALIZER;
#endif

uint32_t BSPLogPortNowMs(void)
{
    return __atomic_load_n(&tick_ms, __ATOMIC_RELAXED);
}

uint8_t BSPLogPortInInterrupt(void)
{
    return interrupt_context;
}

uint8_t BSPLogPortSchedulerRunning(void)
{
    return __atomic_load_n(&scheduler_running, __ATOMIC_RELAXED);
}

uint8_t BSPLogPortIsWorker(void)
{
    return worker_context;
}

uint8_t BSPLogPortInit(void)
{
    return 1u;
}

uint8_t BSPLogPortStartWorker(BSPLogWorkerEntry_t entry)
{
    TEST_CHECK(entry != NULL);
    worker_create_count++;
    return worker_create_success;
}

uint8_t BSPLogPortWriteWholeRecord(const char *data, size_t length)
{
    if (fail_next_write)
    {
        fail_next_write = 0u;
        return 0u;
    }
    if (data == NULL || length == 0u || length >= BSP_LOG_OUTPUT_CAPACITY)
        return 0u;

#if defined(LOG_TEST_THREADS)
    pthread_mutex_lock(&output_lock);
#endif
    if (output_count >= LOG_OUTPUT_CAPACITY)
    {
#if defined(LOG_TEST_THREADS)
        pthread_mutex_unlock(&output_lock);
#endif
        return 0u;
    }
    memcpy(output[output_count], data, length);
    output[output_count][length] = '\0';
    output_lengths[output_count] = length;
    output_count++;
#if defined(LOG_TEST_THREADS)
    pthread_mutex_unlock(&output_lock);
#endif
    return 1u;
}

void BSPLogPortEnterCritical(void)
{
#if defined(LOG_TEST_THREADS)
    pthread_mutex_lock(&log_lock);
#endif
}

void BSPLogPortExitCritical(void)
{
#if defined(LOG_TEST_THREADS)
    pthread_mutex_unlock(&log_lock);
#endif
}

void BSPLogPortDelayMs(uint32_t milliseconds)
{
    (void)milliseconds;
}

#if defined(LOG_TEST_WRAP_FORMAT)
int __real_vsnprintf(char *buffer, size_t length, const char *format, va_list args);
int __wrap_vsnprintf(char *buffer, size_t length, const char *format, va_list args)
{
    (void)buffer;
    (void)length;
    (void)format;
    (void)args;
    format_call_count++;
    return -1;
}
int __wrap_snprintf(char *buffer, size_t length, const char *format, ...)
{
    (void)buffer;
    (void)length;
    (void)format;
    format_call_count++;
    return -1;
}
#if defined(__linux__)
int __wrap___vsnprintf_chk(char *buffer, size_t length, int flag, size_t object_size,
                           const char *format, va_list args)
{
    (void)flag;
    (void)object_size;
    return __wrap_vsnprintf(buffer, length, format, args);
}
int __wrap___snprintf_chk(char *buffer, size_t length, int flag, size_t object_size,
                          const char *format, ...)
{
    (void)buffer;
    (void)length;
    (void)flag;
    (void)object_size;
    (void)format;
    format_call_count++;
    return -1;
}
#endif
#endif

static void test_values(void)
{
    BSPLogState_t state;
    memset(&state, 0xA5, sizeof(state));
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_NOT_INITIALIZED);
    TEST_CHECK(((const uint8_t *)&state)[0] == 0xA5u);
    BSPLogInit();

    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_02, NULL, 0u) == BSP_LOG_STATUS_BAD_ARGUMENT);
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.queue_depth == 1u && state.first_fault_sequence == 1u);

    char text[64];
    memset(text, 'a', sizeof(text));
    BSPLogField_t field = BSP_LOG_FIELD_TEXT(text, sizeof(text));
    TEST_CHECK(BSPLogSubmit(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_02, &field, 1u) == BSP_LOG_STATUS_QUEUED);
    memset(text, 'z', sizeof(text));
    TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_PROCESSED);
    TEST_CHECK(output_count == 1u);
    TEST_CHECK(strstr(output[0], "IMU数据无效") != NULL);
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.shortened_text_fields == 1u && state.queue_depth == 1u);
    TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_PROCESSED);
    TEST_CHECK(output_count == 2u);
    TEST_CHECK(strstr(output[1], "任务名=aaaaaaaa") != NULL);
    TEST_CHECK(strstr(output[1], "text-shortened") != NULL);

    BSPLogHistoryRecord_t history;
    TEST_CHECK(BSPLogGetHistory(2u, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(history.payload_length == BSP_LOG_PAYLOAD_MAX - 46u);
    TEST_CHECK((history.flags & 1u) != 0u);

    interrupt_context = 1u;
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_BAD_CONTEXT);
    interrupt_context = 0u;

    fail_next_write = 1u;
    TEST_CHECK(BSPLogSubmit(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_01, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_PROCESSED);
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.output_failures == 1u && state.queue_depth == 0u);
    TEST_CHECK(BSPLogGetHistory(3u, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(format_call_count == 0u);
}

static void test_history(void)
{
    BSPLogInit();
    tick_ms = UINT32_MAX - 3u;
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_MERGED);
    BSPLogHistoryRecord_t history;
    TEST_CHECK(BSPLogGetHistory(1u, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(history.repeat_count == 2u && history.first_ms == UINT32_MAX - 3u);
    TEST_CHECK(history.last_ms == UINT32_MAX - 3u);

    char text[8];
    BSPLogField_t field;
    for (uint32_t i = 0u; i < 20u; ++i)
    {
        text[0] = (char)('A' + (i % 26u));
        text[1] = (char)('a' + (i / 26u));
        field = BSP_LOG_FIELD_TEXT(text, 2u);
        TEST_CHECK(BSPLogSubmit(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_02,
                                &field, 1u) == BSP_LOG_STATUS_QUEUED);
    }

    /* 最近16条已覆盖序号1；独立保存的首次故障仍可读取。 */
    TEST_CHECK(BSPLogGetHistory(1u, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(history.event_id == EV_MODULES_SAFETY_ROBOT_SAFETY_C_02);
    TEST_CHECK(BSPLogGetHistory(2u, &history) == BSP_LOG_STATUS_NOT_FOUND);
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.merged == 1u && state.history_count == BSP_LOG_HISTORY_CAPACITY);
}

static void test_sequence_wrap(void)
{
    BSPLogInit();
    tick_ms = UINT32_MAX - 4u;
    BSPLogTestSetNextSequence(UINT32_MAX);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    tick_ms = 12u;
    TEST_CHECK(BSPLogSubmit(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_01, NULL, 0u) == BSP_LOG_STATUS_QUEUED);

    BSPLogHistoryRecord_t history;
    TEST_CHECK(BSPLogGetHistory(UINT32_MAX, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(history.first_ms == UINT32_MAX - 4u);
    TEST_CHECK(BSPLogGetHistory(1u, &history) == BSP_LOG_STATUS_OK);
    TEST_CHECK(history.first_ms == 12u);
}

static void test_queue(void)
{
    BSPLogInit();
#if MC02_LOG_ORDINARY
    for (uint32_t i = 0u; i < BSP_LOG_NORMAL_QUEUE_LIMIT; ++i)
    {
        tick_ms = i * 50u;
        BSPLogField_t normal = BSP_LOG_FIELD_U32(i);
        TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &normal, 1u) == BSP_LOG_STATUS_QUEUED);
    }
    tick_ms = 1200u;
    BSPLogField_t full_normal = BSP_LOG_FIELD_U32(99u);
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &full_normal, 1u) == BSP_LOG_STATUS_FULL);

    for (int32_t i = 0; i < (int32_t)BSP_LOG_QUEUE_CAPACITY; ++i)
    {
        BSPLogField_t reason = BSP_LOG_FIELD_I32(i);
        BSPLogStatus_t status = BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04, &reason, 1u);
        TEST_CHECK(status == BSP_LOG_STATUS_QUEUED);
    }
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.queue_depth == BSP_LOG_QUEUE_CAPACITY);
    TEST_CHECK(state.normal_queue_depth == 0u);
    TEST_CHECK(state.evicted_normal == BSP_LOG_NORMAL_QUEUE_LIMIT);

    BSPLogField_t retained = BSP_LOG_FIELD_I32(1000);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04, &retained, 1u) == BSP_LOG_STATUS_RETAINED);
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.queue_depth == BSP_LOG_QUEUE_CAPACITY && state.retained_only == 1u);
#else
    for (int32_t i = 0; i < (int32_t)BSP_LOG_QUEUE_CAPACITY; ++i)
    {
        BSPLogField_t reason = BSP_LOG_FIELD_I32(i);
        TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04, &reason, 1u) == BSP_LOG_STATUS_QUEUED);
    }
    BSPLogField_t retained = BSP_LOG_FIELD_I32(1000);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04, &retained, 1u) == BSP_LOG_STATUS_RETAINED);
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.queue_depth == BSP_LOG_QUEUE_CAPACITY && state.normal_queue_depth == 0u);
    TEST_CHECK(state.retained_only == 1u && state.queue_full == 1u);
    TEST_CHECK(BSPLogGetHistory(state.first_fault_sequence, &(BSPLogHistoryRecord_t){0}) == BSP_LOG_STATUS_OK);
#endif
}

static void make_catalog_fields(const char *types, BSPLogField_t fields[BSP_LOG_MAX_FIELDS], uint8_t *count)
{
    static char sample_text[BSP_LOG_TEXT_MAX + 8u];
    memset(sample_text, 'q', sizeof(sample_text));
    *count = (uint8_t)strlen(types);
    for (uint8_t i = 0u; i < *count; ++i)
    {
        switch (types[i])
        {
        case 'I':
            fields[i] = BSP_LOG_FIELD_I32(-123456);
            break;
        case 'F':
            fields[i] = BSP_LOG_FIELD_F32(1.25f);
            break;
        case 'T':
            fields[i] = BSP_LOG_FIELD_TEXT(sample_text, sizeof(sample_text));
            break;
        case 'H':
        case 'U':
            fields[i] = BSP_LOG_FIELD_U32(0x12345678u);
            break;
        default:
            TEST_CHECK(0);
        }
    }
}

static void test_catalog(void)
{
    BSPLogInit();
    BSPLogField_t fields[BSP_LOG_MAX_FIELDS];
    uint32_t processed = 0u;
#if !MC02_LOG_ORDINARY
    uint32_t filtered = 0u;
#endif
    for (size_t row = 0u; row < sizeof(catalog) / sizeof(catalog[0]); ++row)
    {
        TEST_CHECK(((uint32_t)catalog[row].id >> 8u) == (uint32_t)catalog[row].source);
        size_t type_count = strlen(catalog[row].types);
        size_t label_count = catalog[row].labels[0] == '\0' ? 0u : 1u;
        size_t unit_count = catalog[row].units[0] == '\0' ? 0u : 1u;
        for (const char *cursor = catalog[row].labels; *cursor != '\0'; ++cursor)
            label_count += (*cursor == '|') ? 1u : 0u;
        for (const char *cursor = catalog[row].units; *cursor != '\0'; ++cursor)
            unit_count += (*cursor == '|') ? 1u : 0u;
        if ((catalog[row].labels[0] != '\0' && type_count != label_count) ||
            (catalog[row].units[0] != '\0' && type_count != unit_count))
        {
            (void)fprintf(stderr, "catalog metadata mismatch: id=0x%04x fields=%zu labels=%zu units=%zu\n",
                          (unsigned)catalog[row].id, type_count, label_count, unit_count);
            TEST_CHECK(0);
        }
        for (size_t other = row + 1u; other < sizeof(catalog) / sizeof(catalog[0]); ++other)
            TEST_CHECK(catalog[row].id != catalog[other].id);

        uint8_t count;
        make_catalog_fields(catalog[row].types, fields, &count);
        tick_ms += 50u;
        BSPLogStatus_t status = BSPLogSubmit(catalog[row].id, fields, count);
        if (catalog[row].category == LOG_CATEGORY_NORMAL)
        {
#if MC02_LOG_ORDINARY
            TEST_CHECK(status == BSP_LOG_STATUS_QUEUED);
#else
            TEST_CHECK(status == BSP_LOG_STATUS_FILTERED);
            filtered++;
#endif
        }
        else
        {
            TEST_CHECK(status == BSP_LOG_STATUS_QUEUED);
            TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_PROCESSED);
            processed++;
        }
    }
    TEST_CHECK(output_count == processed);
#if !MC02_LOG_ORDINARY
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.filtered == filtered && filtered > 0u);
#endif
    TEST_CHECK(format_call_count == 0u);
}

static void test_task(void)
{
    BSPLogInit();
    worker_create_success = 0u;
    TEST_CHECK(BSPLogTaskInit() == BSP_LOG_STATUS_TASK_CREATE_FAILED);
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.task_create_failures == 1u && state.task_started == 0u);

    worker_create_success = 1u;
    TEST_CHECK(BSPLogTaskInit() == BSP_LOG_STATUS_OK);
    TEST_CHECK(BSPLogTaskInit() == BSP_LOG_STATUS_OK);
    TEST_CHECK(worker_create_count == 2u);
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_02, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogProcessPending(2u) == BSP_LOG_STATUS_BAD_CONTEXT);
    TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_PROCESSED);

    scheduler_running = 1u;
    TEST_CHECK(BSPLogProcessPending(1u) == BSP_LOG_STATUS_BAD_CONTEXT);
    worker_context = 1u;
    TEST_CHECK(BSPLogProcessPending(2u) == BSP_LOG_STATUS_EMPTY);
    worker_context = 0u;
    scheduler_running = 0u;
}

#if MC02_LOG_ORDINARY
static void test_ordinary(void)
{
    BSPLogInit();
    for (uint32_t i = 0u; i < 8u; ++i)
    {
        BSPLogField_t field = BSP_LOG_FIELD_U32(i);
        TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &field, 1u) == BSP_LOG_STATUS_QUEUED);
    }
    BSPLogField_t field = BSP_LOG_FIELD_U32(8u);
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &field, 1u) == BSP_LOG_STATUS_RATE_LIMITED);
    tick_ms = 50u;
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &field, 1u) == BSP_LOG_STATUS_QUEUED);
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.queue_depth == 9u && state.normal_queue_depth == 9u);
    TEST_CHECK(state.rate_limited == 1u);
}

static void test_fairness(void)
{
    BSPLogInit();
    BSPLogField_t normal_a = BSP_LOG_FIELD_U32(1u);
    BSPLogField_t normal_b = BSP_LOG_FIELD_U32(2u);
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &normal_a, 1u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogSubmit(EV_BSP_USB_BSP_USB_C_01, &normal_b, 1u) == BSP_LOG_STATUS_QUEUED);
    TEST_CHECK(BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02, NULL, 0u) == BSP_LOG_STATUS_QUEUED);
    scheduler_running = 1u;
    worker_context = 1u;
    TEST_CHECK(BSPLogProcessPending(2u) == BSP_LOG_STATUS_PROCESSED);
    worker_context = 0u;
    scheduler_running = 0u;
    TEST_CHECK(output_count == 2u);
    TEST_CHECK(strstr(output[0], "IMU数据无效") != NULL);
    TEST_CHECK(strstr(output[1], "USB") != NULL);
}

static void test_format_overflow(void)
{
    char long_text[BSP_LOG_TEXT_MAX];
    memset(long_text, 'x', sizeof(long_text));
    BSPLogField_t fields[8] = {
        BSP_LOG_FIELD_TEXT(long_text, sizeof(long_text)), BSP_LOG_FIELD_U32(UINT32_MAX),
        BSP_LOG_FIELD_TEXT(long_text, sizeof(long_text)), BSP_LOG_FIELD_U32(UINT32_MAX),
        BSP_LOG_FIELD_TEXT(long_text, sizeof(long_text)), BSP_LOG_FIELD_U32(UINT32_MAX),
        BSP_LOG_FIELD_TEXT(long_text, sizeof(long_text)), BSP_LOG_FIELD_U32(UINT32_MAX),
    };

    BSPLogInit();
    TEST_CHECK(BSPLogSubmit(EV_MODULES_LCD_LCD_STATS_C_01, fields, 8u) == BSP_LOG_STATUS_QUEUED);
    scheduler_running = 1u;
    worker_context = 1u;
    TEST_CHECK(BSPLogProcessPending(2u) == BSP_LOG_STATUS_PROCESSED);
    worker_context = 0u;
    scheduler_running = 0u;

    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(output_count == 0u);
    TEST_CHECK(state.format_failures == 1u && state.output_failures == 0u);
}
#endif

#if defined(LOG_TEST_THREADS)
typedef struct
{
    uint32_t base;
} ProducerArgs_t;
static atomic_uint producers_done;
static atomic_uint thread_failure;

static void *producer(void *argument)
{
    ProducerArgs_t *args = (ProducerArgs_t *)argument;
    for (uint32_t i = 0u; i < 100u; ++i)
    {
        BSPLogField_t field = BSP_LOG_FIELD_I32((int32_t)(args->base + i));
        BSPLogStatus_t status = BSPLogSubmit(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04, &field, 1u);
        if (status != BSP_LOG_STATUS_QUEUED && status != BSP_LOG_STATUS_RETAINED &&
            status != BSP_LOG_STATUS_MERGED)
            atomic_store(&thread_failure, 1u);
    }
    atomic_fetch_add(&producers_done, 1u);
    return NULL;
}

static void *processor(void *argument)
{
    (void)argument;
    worker_context = 1u;
    scheduler_running = 1u;
    while (atomic_load(&producers_done) != 4u)
    {
        (void)BSPLogProcessPending(2u);
        BSPLogState_t state;
        if (BSPLogGetState(&state) != BSP_LOG_STATUS_OK || state.queue_depth > BSP_LOG_QUEUE_CAPACITY)
            atomic_store(&thread_failure, 1u);
    }
    while (BSPLogProcessPending(2u) == BSP_LOG_STATUS_PROCESSED)
        ;
    scheduler_running = 0u;
    worker_context = 0u;
    return NULL;
}

static void test_threads(void)
{
    BSPLogInit();
    pthread_t workers[4];
    pthread_t drain;
    ProducerArgs_t args[4];
    TEST_CHECK(pthread_create(&drain, NULL, processor, NULL) == 0);
    for (uint32_t i = 0u; i < 4u; ++i)
    {
        args[i].base = (i + 1u) * 1000u;
        TEST_CHECK(pthread_create(&workers[i], NULL, producer, &args[i]) == 0);
    }
    for (uint32_t i = 0u; i < 4u; ++i)
        TEST_CHECK(pthread_join(workers[i], NULL) == 0);
    TEST_CHECK(pthread_join(drain, NULL) == 0);
    TEST_CHECK(atomic_load(&thread_failure) == 0u);
    BSPLogState_t state;
    TEST_CHECK(BSPLogGetState(&state) == BSP_LOG_STATUS_OK);
    TEST_CHECK(state.submitted == 400u && state.queue_depth == 0u);
    TEST_CHECK(state.first_fault_sequence != 0u && state.history_count == BSP_LOG_HISTORY_CAPACITY);
}
#endif

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "values") == 0)
        test_values();
    else if (strcmp(argv[1], "history") == 0)
        test_history();
    else if (strcmp(argv[1], "sequence") == 0)
        test_sequence_wrap();
    else if (strcmp(argv[1], "queue") == 0)
        test_queue();
    else if (strcmp(argv[1], "catalog") == 0)
        test_catalog();
    else if (strcmp(argv[1], "task") == 0)
        test_task();
#if MC02_LOG_ORDINARY
    else if (strcmp(argv[1], "ordinary") == 0)
        test_ordinary();
    else if (strcmp(argv[1], "fairness") == 0)
        test_fairness();
    else if (strcmp(argv[1], "format-overflow") == 0)
        test_format_overflow();
#endif
#if defined(LOG_TEST_THREADS)
    else if (strcmp(argv[1], "threads") == 0)
        test_threads();
#endif
    else
        TEST_CHECK(0);
    return EXIT_SUCCESS;
}
