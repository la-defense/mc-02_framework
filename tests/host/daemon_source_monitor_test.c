/** @file daemon_source_monitor_test.c
 * @brief 使用生产 Daemon 接口验证来源期限、事件队列和并发读取。
 * @note 每个场景由独立进程运行；HAL 时钟和 CMSIS 任务创建由本测试建模。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "daemon.h"
#include "cmsis_os.h"
#include "platform_os.h"
#include "test_check.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static _Atomic uint32_t test_tick;
static unsigned task_create_calls;
static unsigned task_create_failures;
static osThreadDef_t last_task_definition;
static DaemonEvent_t received_events[16];
static unsigned received_event_count;
static unsigned consumer_one_events;
static unsigned consumer_two_events;
static _Atomic unsigned thread_finished;

static void PlatformTestEntry(void const *argument)
{
    (void)argument; // 替身只检查任务入口，不实际启动它。
}

static void ScenarioPlatformTaskValidation(void)
{
    PlatformTaskConfig_t config = {
        .name = "platform_test", // 验证传给后端的任务名称。
        .entry = PlatformTestEntry, // 使用不会执行实际工作的入口。
        .argument = NULL, // 此场景不需要入口参数。
        .stack_words = 128u, // 测试栈容量按 FreeRTOS 栈字给出。
        .priority = PLATFORM_TASK_NORMAL, // 验证普通优先级的映射。
    };
    TEST_CHECK(PlatformTaskCreate(NULL) == 0u); // 空配置必须在调用后端前拒绝。
    config.stack_words = 0u; // 零栈不能创建任务。
    TEST_CHECK(PlatformTaskCreate(&config) == 0u && task_create_calls == 0u);
    config.stack_words = (uint32_t)UINT16_MAX + 1u; // FreeRTOS 栈深类型容不下时不得静默截断。
    TEST_CHECK(PlatformTaskCreate(&config) == 0u && task_create_calls == 0u);
    config.stack_words = 128u; // 恢复有效栈容量。
    config.priority = (PlatformTaskPriority_t)99; // 未声明优先级不能映射到后端。
    TEST_CHECK(PlatformTaskCreate(&config) == 0u && task_create_calls == 0u);
    config.priority = PLATFORM_TASK_NORMAL; // 有效配置应成功提交。
    TEST_CHECK(PlatformTaskCreate(&config) == 1u && task_create_calls == 1u);
    TEST_CHECK(last_task_definition.tpriority == osPriorityNormal && last_task_definition.stacksize == 128u);
    config.priority = PLATFORM_TASK_ABOVE_NORMAL; // 检查较高优先级不会被当作普通任务。
    TEST_CHECK(PlatformTaskCreate(&config) == 1u && last_task_definition.tpriority == osPriorityAboveNormal);
    config.priority = PLATFORM_TASK_LOW; // LCD 优先级应保持低于普通任务。
    PlatformTaskHandle_t handle = NULL;
    TEST_CHECK(PlatformTaskCreateTracked(&config, &handle) == 1u && handle != NULL);
    TEST_CHECK(last_task_definition.tpriority == osPriorityLow);
    PlatformTaskHandle_t previous = handle;
    task_create_failures = 1u; // 模拟后端拒绝，不能覆盖之前保存的句柄。
    TEST_CHECK(PlatformTaskCreateTracked(&config, &handle) == 0u && handle == previous);
    TEST_CHECK(PlatformTaskCreateTracked(&config, NULL) == 0u);
    PlatformTaskDelayMs(7u); // 等待值原样传给后端，避免默默改变单位。
    TEST_CHECK(atomic_load_explicit(&test_tick, memory_order_relaxed) == 7u);
}

uint32_t HAL_GetTick(void)
{
    return atomic_load_explicit(&test_tick, memory_order_relaxed);
}

osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument)
{
    (void)argument;
    TEST_CHECK(definition != NULL);
    ++task_create_calls;
    last_task_definition = *definition;
    if (task_create_failures != 0u)
    {
        --task_create_failures;
        return NULL;
    }
    return (osThreadId)(uintptr_t)1u;
}

void osDelay(uint32_t milliseconds)
{
    atomic_fetch_add_explicit(&test_tick, milliseconds, memory_order_relaxed);
}

static void CaptureEvent(const DaemonEvent_t *event, void *context)
{
    (void)context;
    TEST_CHECK(event != NULL);
    TEST_CHECK(received_event_count < (sizeof(received_events) / sizeof(received_events[0])));
    received_events[received_event_count++] = *event;
}

static void ConsumerOne(const DaemonEvent_t *event, void *context)
{
    (void)context;
    TEST_CHECK(event != NULL);
    ++consumer_one_events;
}

static void ConsumerTwo(const DaemonEvent_t *event, void *context)
{
    (void)context;
    TEST_CHECK(event != NULL);
    ++consumer_two_events;
}

static Daemon_Init_Config_s MakeConfig(uint16_t source_id, uint32_t transport_ms,
                                       uint32_t data_ms, uint32_t startup_ms,
                                       uint32_t event_age_ms, uint8_t capacity,
                                       DaemonEventPolicy_t policy, uint8_t consumer_id,
                                       DaemonConsumerMode_t consumer_mode,
                                       void (*handler)(const DaemonEvent_t *, void *))
{
    const Daemon_Init_Config_s config = {
        .transport_timeout_ms = transport_ms,
        .data_timeout_ms = data_ms,
        .startup_grace_ms = startup_ms,
        .event_max_age_ms = event_age_ms,
        .source_id = source_id,
        .event_capacity = capacity,
        .consumer_id = consumer_id,
        .event_policy = policy,
        .consumer_mode = consumer_mode,
        .event_handler = handler,
    };
    return config;
}

static DaemonSourceStatus_t ReadStatus(DaemonInstance *instance, uint32_t now_ms)
{
    DaemonSourceStatus_t status = {0};
    TEST_CHECK(DaemonGetSourceStatus(instance, now_ms, &status) == DAEMON_OK);
    return status;
}

static void ScenarioRegistrationAndDeadlines(void)
{
    atomic_store(&test_tick, 0u);
    Daemon_Init_Config_s config = MakeConfig(1u, 20u, 10u, 5u, 100u, 8u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    TEST_CHECK(DaemonRegister(NULL) == NULL);
    Daemon_Init_Config_s invalid = config;
    invalid.transport_timeout_ms = 0u;
    TEST_CHECK(DaemonRegister(&invalid) == NULL);
    invalid = config;
    invalid.consumer_id = DAEMON_EVENT_TASK_CONSUMER;
    TEST_CHECK(DaemonRegister(&invalid) == NULL);

    DaemonInstance *instance = DaemonRegister(&config);
    TEST_CHECK(instance != NULL);
    TEST_CHECK(ReadStatus(instance, 5u).state == DAEMON_SOURCE_WAITING_FIRST_FRAME);
    TEST_CHECK(ReadStatus(instance, 6u).state == DAEMON_SOURCE_OFFLINE);
    atomic_store(&test_tick, 6u);
    DaemonTask();
    uint8_t processed = 0u;
    TEST_CHECK(DaemonProcessTaskEvents(1u, 8u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 1u && received_events[0].state == DAEMON_SOURCE_OFFLINE);

    atomic_store(&test_tick, 10u);
    TEST_CHECK(DaemonReportAt(instance, 10u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    DaemonSourceStatus_t status = ReadStatus(instance, 20u);
    TEST_CHECK(status.state == DAEMON_SOURCE_ONLINE_DATA_VALID);
    TEST_CHECK(status.transport_age_ms == 10u && status.data_age_ms == 10u);
    TEST_CHECK(status.transport_deadline_ms == 30u && status.data_deadline_ms == 20u);
    TEST_CHECK(ReadStatus(instance, 21u).state == DAEMON_SOURCE_ONLINE_DATA_INVALID);
    TEST_CHECK(ReadStatus(instance, 30u).state == DAEMON_SOURCE_ONLINE_DATA_INVALID);
    TEST_CHECK(ReadStatus(instance, 31u).state == DAEMON_SOURCE_OFFLINE);

    atomic_store(&test_tick, 32u);
    TEST_CHECK(DaemonReportAt(instance, 32u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT) == DAEMON_OK);
    status = ReadStatus(instance, 32u);
    TEST_CHECK(status.transport_online != 0u && status.data_valid == 0u);
    TEST_CHECK(status.last_data_rx_ms == 32u);

    atomic_store(&test_tick, 55u);
    TEST_CHECK(DaemonReportAt(instance, 33u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_ERR_EXPIRED);
    TEST_CHECK(DaemonReportAt(instance, 55u, 0u) == DAEMON_OK);
    status = ReadStatus(instance, 55u);
    TEST_CHECK(status.last_transport_rx_ms == 32u && status.last_data_rx_ms == 32u);
    TEST_CHECK(status.state == DAEMON_SOURCE_OFFLINE);

    DaemonSourceStatus_t unchanged = {.state = DAEMON_SOURCE_ONLINE_DATA_VALID,
                                      .last_transport_rx_ms = 0x12345678u};
    DaemonInstance unregistered = {0};
    TEST_CHECK(DaemonGetSourceStatus(&unregistered, 0u, &unchanged) == DAEMON_ERR_NOT_REGISTERED);
    TEST_CHECK(unchanged.state == DAEMON_SOURCE_ONLINE_DATA_VALID &&
               unchanged.last_transport_rx_ms == 0x12345678u);
}

static void ScenarioClockWrap(void)
{
    atomic_store(&test_tick, UINT32_MAX - 5u);
    Daemon_Init_Config_s config = MakeConfig(2u, 10u, 10u, 10u, 20u, 2u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *instance = DaemonRegister(&config);
    TEST_CHECK(instance != NULL);
    TEST_CHECK(ReadStatus(instance, 3u).state == DAEMON_SOURCE_WAITING_FIRST_FRAME);
    TEST_CHECK(ReadStatus(instance, 4u).state == DAEMON_SOURCE_WAITING_FIRST_FRAME);
    TEST_CHECK(ReadStatus(instance, 5u).state == DAEMON_SOURCE_OFFLINE);

    atomic_store(&test_tick, 10u);
    TEST_CHECK(DaemonReportAt(instance, 10u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    TEST_CHECK(ReadStatus(instance, 20u).state == DAEMON_SOURCE_ONLINE_DATA_VALID);
    TEST_CHECK(ReadStatus(instance, 21u).state == DAEMON_SOURCE_OFFLINE);
    TEST_CHECK(DaemonReportAt(instance, 9u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_ERR_ARGUMENT);
}

static void ScenarioFifoAndOverflow(void)
{
    atomic_store(&test_tick, 0u);
    Daemon_Init_Config_s fifo_config = MakeConfig(3u, 10u, 10u, 100u, 10u, 3u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *fifo = DaemonRegister(&fifo_config);
    TEST_CHECK(fifo != NULL);
    TEST_CHECK(DaemonReportAt(fifo, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    atomic_store(&test_tick, 1u);
    TEST_CHECK(DaemonReportAt(fifo, 1u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT) == DAEMON_OK);
    atomic_store(&test_tick, 2u);
    uint8_t processed = 0u;
    TEST_CHECK(DaemonProcessTaskEvents(1u, 3u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 2u && received_event_count == 2u);
    TEST_CHECK(received_events[0].previous_state == DAEMON_SOURCE_WAITING_FIRST_FRAME &&
               received_events[0].state == DAEMON_SOURCE_ONLINE_DATA_VALID);
    TEST_CHECK(received_events[1].previous_state == DAEMON_SOURCE_ONLINE_DATA_VALID &&
               received_events[1].state == DAEMON_SOURCE_ONLINE_DATA_INVALID);

    Daemon_Init_Config_s limited_config = MakeConfig(4u, 10u, 10u, 100u, 10u, 1u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *limited = DaemonRegister(&limited_config);
    TEST_CHECK(limited != NULL);
    TEST_CHECK(DaemonReportAt(limited, 2u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    atomic_store(&test_tick, 3u);
    TEST_CHECK(DaemonReportAt(limited, 3u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT) == DAEMON_OK);
    DaemonStats_t stats = {0};
    TEST_CHECK(DaemonGetStats(limited, &stats) == DAEMON_OK);
    TEST_CHECK(stats.queued == 1u && stats.overflowed == 1u && stats.enqueued == 1u);
    TEST_CHECK(DaemonProcessTaskEvents(1u, 0u, &processed) == DAEMON_ERR_ARGUMENT);
}

static void ScenarioEventPolicies(void)
{
    atomic_store(&test_tick, 0u);
    Daemon_Init_Config_s count_config = MakeConfig(5u, 20u, 20u, 100u, 20u, 1u,
        DAEMON_EVENT_COUNT, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *count_instance = DaemonRegister(&count_config);
    TEST_CHECK(count_instance != NULL);
    TEST_CHECK(DaemonReportAt(count_instance, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    atomic_store(&test_tick, 1u);
    TEST_CHECK(DaemonReportAt(count_instance, 1u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT) == DAEMON_OK);
    DaemonStats_t stats = {0};
    TEST_CHECK(DaemonGetStats(count_instance, &stats) == DAEMON_OK);
    TEST_CHECK(stats.queued == 1u && stats.enqueued == 2u && stats.overflowed == 0u);
    uint8_t processed = 0u;
    TEST_CHECK(DaemonProcessTaskEvents(1u, 1u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 1u && received_events[0].occurrences == 2u);
    TEST_CHECK(received_events[0].detected_ms == 0u && received_events[0].last_detected_ms == 1u);
    TEST_CHECK(received_events[0].previous_state == DAEMON_SOURCE_WAITING_FIRST_FRAME &&
               received_events[0].state == DAEMON_SOURCE_ONLINE_DATA_INVALID);

    Daemon_Init_Config_s coalesced_config = MakeConfig(6u, 20u, 20u, 100u, 20u, 1u,
        DAEMON_EVENT_COALESCED_STATE, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *coalesced = DaemonRegister(&coalesced_config);
    TEST_CHECK(coalesced != NULL);
    atomic_store(&test_tick, 1u);
    TEST_CHECK(DaemonReportAt(coalesced, 1u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    atomic_store(&test_tick, 2u);
    TEST_CHECK(DaemonReportAt(coalesced, 2u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT) == DAEMON_OK);
    TEST_CHECK(DaemonGetStats(coalesced, &stats) == DAEMON_OK);
    TEST_CHECK(stats.queued == 1u && stats.enqueued == 2u);
    TEST_CHECK(DaemonProcessTaskEvents(1u, 1u, &processed) == DAEMON_OK);
    TEST_CHECK(received_events[1].sequence == 2u && received_events[1].occurrences == 1u);
    TEST_CHECK(received_events[1].previous_state == DAEMON_SOURCE_ONLINE_DATA_VALID &&
               received_events[1].state == DAEMON_SOURCE_ONLINE_DATA_INVALID);
}

static void ScenarioExpiration(void)
{
    atomic_store(&test_tick, 0u);
    Daemon_Init_Config_s config = MakeConfig(7u, 10u, 10u, 100u, 2u, 2u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, CaptureEvent);
    DaemonInstance *instance = DaemonRegister(&config);
    TEST_CHECK(instance != NULL);
    TEST_CHECK(DaemonReportAt(instance, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    atomic_store(&test_tick, 3u);
    uint8_t processed = 9u;
    TEST_CHECK(DaemonProcessTaskEvents(1u, 2u, &processed) == DAEMON_ERR_EXPIRED);
    TEST_CHECK(processed == 0u && received_event_count == 0u);
    DaemonStats_t stats = {0};
    TEST_CHECK(DaemonGetStats(instance, &stats) == DAEMON_OK);
    TEST_CHECK(stats.expired == 1u && stats.queued == 0u && stats.global_expired == 1u);
}

static void ScenarioConsumersAndIndependentTask(void)
{
    atomic_store(&test_tick, 0u);
    Daemon_Init_Config_s first_config = MakeConfig(8u, 10u, 10u, 100u, 20u, 2u,
        DAEMON_EVENT_FIFO, 1u, DAEMON_CONSUMER_BOUND_TASK, ConsumerOne);
    Daemon_Init_Config_s second_config = MakeConfig(9u, 10u, 10u, 100u, 20u, 2u,
        DAEMON_EVENT_FIFO, 2u, DAEMON_CONSUMER_BOUND_TASK, ConsumerTwo);
    DaemonInstance *first = DaemonRegister(&first_config);
    DaemonInstance *second = DaemonRegister(&second_config);
    TEST_CHECK(first != NULL && second != NULL);
    TEST_CHECK(DaemonReportAt(first, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    TEST_CHECK(DaemonReportAt(second, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    uint8_t processed = 0u;
    TEST_CHECK(DaemonProcessTaskEvents(2u, 2u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 1u && consumer_one_events == 0u && consumer_two_events == 1u);
    // 事件消费开始后，实例地址已可能被处理路径读取，不能再注销。
    TEST_CHECK(DaemonUnregister(first) == 0u);
    TEST_CHECK(DaemonProcessTaskEvents(1u, 2u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 1u && consumer_one_events == 1u && consumer_two_events == 1u);

    Daemon_Init_Config_s independent_config = MakeConfig(10u, 10u, 10u, 0u, 20u, 2u,
        DAEMON_EVENT_FIFO, DAEMON_EVENT_TASK_CONSUMER, DAEMON_CONSUMER_INDEPENDENT_TASK,
        CaptureEvent);
    DaemonInstance *independent = DaemonRegister(&independent_config);
    TEST_CHECK(independent != NULL);
    TEST_CHECK(DaemonProcessTaskEvents(DAEMON_EVENT_TASK_CONSUMER, 2u, &processed) ==
               DAEMON_ERR_CONSUMER);
    task_create_failures = 1u;
    TEST_CHECK(DaemonEventTaskInit() == DAEMON_ERR_TASK_CREATE);
    DaemonStats_t stats = {0};
    TEST_CHECK(DaemonGetStats(NULL, &stats) == DAEMON_OK);
    TEST_CHECK(stats.task_started == 0u && stats.task_failed == 1u);
    TEST_CHECK(DaemonEventTaskInit() == DAEMON_OK);
    TEST_CHECK(DaemonEventTaskInit() == DAEMON_OK);
    TEST_CHECK(task_create_calls == 2u && last_task_definition.stacksize == 512u);
    TEST_CHECK(last_task_definition.tpriority == osPriorityBelowNormal);
    TEST_CHECK(DaemonGetStats(NULL, &stats) == DAEMON_OK && stats.task_started == 1u &&
               stats.task_failed == 0u);
    TEST_CHECK(DaemonReportAt(independent, 0u, DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID) == DAEMON_OK);
    TEST_CHECK(DaemonProcessTaskEvents(DAEMON_EVENT_TASK_CONSUMER, 2u, &processed) == DAEMON_OK);
    TEST_CHECK(processed == 1u && received_event_count == 1u);
}

static void *ProducerThread(void *argument)
{
    DaemonInstance *instance = (DaemonInstance *)argument;
    for (uint32_t timestamp = 1u; timestamp <= 50000u; ++timestamp)
    {
        atomic_store_explicit(&test_tick, timestamp, memory_order_relaxed);
        const DaemonStatus_t result = DaemonReportAt(instance, timestamp,
            DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT |
            DAEMON_REPORT_DATA_VALID);
        TEST_CHECK(result == DAEMON_OK);
    }
    atomic_store_explicit(&thread_finished, 1u, memory_order_release);
    return NULL;
}

static void *ReaderThread(void *argument)
{
    DaemonInstance *instance = (DaemonInstance *)argument;
    while (atomic_load_explicit(&thread_finished, memory_order_acquire) == 0u)
    {
        const uint32_t now_ms = HAL_GetTick();
        DaemonSourceStatus_t status = {0};
        TEST_CHECK(DaemonGetSourceStatus(instance, now_ms, &status) == DAEMON_OK);
        TEST_CHECK(status.has_transport_frame <= 1u && status.has_data_sample <= 1u);
        TEST_CHECK(status.data_valid == 0u ||
                   (status.transport_online != 0u && status.has_data_sample != 0u));
        DaemonStats_t stats = {0};
        TEST_CHECK(DaemonGetStats(instance, &stats) == DAEMON_OK);
        TEST_CHECK(stats.queued <= 1u);
        DaemonTask();
    }
    return NULL;
}

static void ScenarioConcurrentAccess(void)
{
    atomic_store(&test_tick, 0u);
    atomic_store(&thread_finished, 0u);
    Daemon_Init_Config_s config = MakeConfig(11u, 100u, 100u, 10u, 100u, 1u,
        DAEMON_EVENT_COALESCED_STATE, 1u, DAEMON_CONSUMER_BOUND_TASK, NULL);
    DaemonInstance *instance = DaemonRegister(&config);
    TEST_CHECK(instance != NULL);
    pthread_t producer;
    pthread_t reader;
    TEST_CHECK(pthread_create(&producer, NULL, ProducerThread, instance) == 0);
    TEST_CHECK(pthread_create(&reader, NULL, ReaderThread, instance) == 0);
    TEST_CHECK(pthread_join(producer, NULL) == 0);
    TEST_CHECK(pthread_join(reader, NULL) == 0);
    const DaemonSourceStatus_t status = ReadStatus(instance, 50000u);
    TEST_CHECK(status.state == DAEMON_SOURCE_ONLINE_DATA_VALID);
    TEST_CHECK(status.last_transport_rx_ms == 50000u && status.last_data_rx_ms == 50000u);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "platform-task") == 0)
        ScenarioPlatformTaskValidation();
    else if (strcmp(argv[1], "deadlines") == 0)
        ScenarioRegistrationAndDeadlines();
    else if (strcmp(argv[1], "wrap") == 0)
        ScenarioClockWrap();
    else if (strcmp(argv[1], "fifo-overflow") == 0)
        ScenarioFifoAndOverflow();
    else if (strcmp(argv[1], "policies") == 0)
        ScenarioEventPolicies();
    else if (strcmp(argv[1], "expiration") == 0)
        ScenarioExpiration();
    else if (strcmp(argv[1], "consumers-task-retry") == 0)
        ScenarioConsumersAndIndependentTask();
    else if (strcmp(argv[1], "threads") == 0)
        ScenarioConcurrentAccess();
    else
        TEST_CHECK(0);
    return EXIT_SUCCESS;
}
