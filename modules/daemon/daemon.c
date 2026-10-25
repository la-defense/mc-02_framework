/** @file daemon.c
 * @brief 计算来源绝对期限并把状态变化排入每来源的固定队列。
 * @note 所有状态复制和队列修改都在短临界区内完成；模块事件回调在区外执行。
 */
#include "daemon.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

static DaemonInstance *daemon_instances[DAEMON_MX_CNT];
static uint8_t daemon_count;
static uint8_t daemon_task_ran;
static uint8_t daemon_event_consumer_ran;
static uint8_t event_task_started;
static uint8_t event_task_failed;
static uint32_t global_event_overflow;
static uint32_t global_event_expired;

static DaemonStatus_t process_independent_events(uint8_t max_events, uint8_t *processed);

static uint32_t saturating_increment(uint32_t value)
{
    return value == UINT32_MAX ? value : value + 1u;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static uint8_t deadline_valid(uint32_t duration_ms)
{
    return (uint8_t)(duration_ms != 0u && duration_ms <= INT32_MAX);
}

static uint8_t instance_registered_locked(const DaemonInstance *instance)
{
    for (uint8_t i = 0u; i < daemon_count; ++i)
        if (daemon_instances[i] == instance)
            return 1u;
    return 0u;
}

static DaemonSourceState_t source_state_locked(const DaemonInstance *instance, uint32_t now_ms)
{
    if (instance->has_transport_frame == 0u)
        return elapsed_ms(now_ms, instance->registered_ms) <= instance->startup_grace_ms ?
            DAEMON_SOURCE_WAITING_FIRST_FRAME : DAEMON_SOURCE_OFFLINE;

    if (elapsed_ms(now_ms, instance->last_transport_rx_ms) > instance->transport_timeout_ms)
        return DAEMON_SOURCE_OFFLINE;

    if (instance->has_data_sample == 0u || instance->data_valid == 0u ||
        elapsed_ms(now_ms, instance->last_data_rx_ms) > instance->data_timeout_ms)
        return DAEMON_SOURCE_ONLINE_DATA_INVALID;

    return DAEMON_SOURCE_ONLINE_DATA_VALID;
}

static void queue_remove_locked(DaemonInstance *instance, uint8_t index)
{
    for (uint8_t i = (uint8_t)(index + 1u); i < instance->event_count; ++i)
    {
        const uint8_t dst = (uint8_t)((instance->event_head + i - 1u) % DAEMON_EVENT_MAX_DEPTH);
        const uint8_t src = (uint8_t)((instance->event_head + i) % DAEMON_EVENT_MAX_DEPTH);
        instance->events[dst] = instance->events[src];
    }
    instance->event_count--;
}

static void enqueue_state_locked(DaemonInstance *instance, DaemonSourceState_t previous,
                                 DaemonSourceState_t current, uint32_t detected_ms)
{
    DaemonEvent_t candidate = {0};
    candidate.source_id = instance->source_id;
    candidate.sequence = ++instance->next_event_sequence;
    if (candidate.sequence == 0u)
        candidate.sequence = ++instance->next_event_sequence;
    candidate.detected_ms = detected_ms;
    candidate.last_detected_ms = detected_ms;
    candidate.occurrences = 1u;
    candidate.previous_state = previous;
    candidate.state = current;

    if (instance->event_policy == DAEMON_EVENT_COALESCED_STATE)
    {
        instance->event_head = 0u;
        instance->events[0] = candidate;
        instance->event_count = 1u;
    }
    else if (instance->event_policy == DAEMON_EVENT_COUNT)
    {
        if (instance->event_count != 0u)
        {
            DaemonEvent_t *existing = &instance->events[instance->event_head];
            existing->occurrences = saturating_increment(existing->occurrences);
            existing->last_detected_ms = detected_ms;
            existing->state = current;
            goto recorded;
        }
        if (instance->event_count < instance->event_capacity)
        {
            const uint8_t tail = (uint8_t)((instance->event_head + instance->event_count) %
                                           DAEMON_EVENT_MAX_DEPTH);
            instance->events[tail] = candidate;
            instance->event_count++;
        }
        else
            goto overflow;
    }
    else if (instance->event_count < instance->event_capacity)
    {
        const uint8_t tail = (uint8_t)((instance->event_head + instance->event_count) %
                                       DAEMON_EVENT_MAX_DEPTH);
        instance->events[tail] = candidate;
        instance->event_count++;
    }
    else
        goto overflow;

recorded:
    instance->enqueued = saturating_increment(instance->enqueued);
    return;

overflow:
    instance->overflowed = saturating_increment(instance->overflowed);
    global_event_overflow = saturating_increment(global_event_overflow);
}

static void update_state_locked(DaemonInstance *instance, uint32_t now_ms)
{
    const DaemonSourceState_t current = source_state_locked(instance, now_ms);
    if (current == instance->published_state)
        return;
    enqueue_state_locked(instance, instance->published_state, current, now_ms);
    instance->published_state = current;
}

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    if (config == NULL || !deadline_valid(config->transport_timeout_ms) ||
        !deadline_valid(config->data_timeout_ms) || config->startup_grace_ms > INT32_MAX ||
        !deadline_valid(config->event_max_age_ms) || config->source_id == 0u ||
        config->event_capacity == 0u || config->event_capacity > DAEMON_EVENT_MAX_DEPTH ||
        config->event_policy < DAEMON_EVENT_FIFO ||
        config->event_policy > DAEMON_EVENT_COALESCED_STATE ||
        config->consumer_mode > DAEMON_CONSUMER_INDEPENDENT_TASK ||
        (config->consumer_mode == DAEMON_CONSUMER_BOUND_TASK &&
         (config->consumer_id == 0u || config->consumer_id == DAEMON_EVENT_TASK_CONSUMER)) ||
        (config->consumer_mode == DAEMON_CONSUMER_INDEPENDENT_TASK && config->event_handler == NULL) ||
        (config->event_policy != DAEMON_EVENT_FIFO && config->event_capacity != 1u))
        return NULL;

    DaemonInstance *instance = (DaemonInstance *)calloc(1u, sizeof(*instance));
    if (instance == NULL)
        return NULL;

    instance->transport_timeout_ms = config->transport_timeout_ms;
    instance->data_timeout_ms = config->data_timeout_ms;
    instance->startup_grace_ms = config->startup_grace_ms;
    instance->event_max_age_ms = config->event_max_age_ms;
    instance->source_id = config->source_id;
    instance->event_capacity = config->event_capacity;
    instance->consumer_id = config->consumer_id;
    instance->consumer_mode = (uint8_t)config->consumer_mode;
    instance->event_policy = (uint8_t)config->event_policy;
    instance->event_handler = config->event_handler;
    instance->owner_id = config->owner_id;
    instance->registered_ms = HAL_GetTick();
    instance->published_state = DAEMON_SOURCE_WAITING_FIRST_FRAME;

    taskENTER_CRITICAL();
    if (daemon_count >= DAEMON_MX_CNT || daemon_task_ran != 0u || event_task_started != 0u)
    {
        taskEXIT_CRITICAL();
        free(instance);
        return NULL;
    }
    daemon_instances[daemon_count++] = instance;
    taskEXIT_CRITICAL();
    return instance;
}

uint8_t DaemonUnregister(DaemonInstance *instance)
{
    if (instance == NULL)
        return 0u;

    uint8_t removed = 0u;
    taskENTER_CRITICAL();
    if (daemon_task_ran == 0u && daemon_event_consumer_ran == 0u && event_task_started == 0u)
    {
        for (uint8_t i = 0u; i < daemon_count; ++i)
        {
            if (daemon_instances[i] != instance)
                continue;
            for (uint8_t j = (uint8_t)(i + 1u); j < daemon_count; ++j)
                daemon_instances[j - 1u] = daemon_instances[j];
            daemon_instances[--daemon_count] = NULL;
            removed = 1u;
            break;
        }
    }
    taskEXIT_CRITICAL();

    if (removed != 0u)
        free(instance);
    return removed;
}

DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{
    if (instance == NULL || (flags & (uint8_t)~(DAEMON_REPORT_TRANSPORT_VALID |
        DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID)) != 0u ||
        ((flags & DAEMON_REPORT_DATA_VALID) != 0u && (flags & DAEMON_REPORT_DATA_PRESENT) == 0u))
        return DAEMON_ERR_ARGUMENT;
    taskENTER_CRITICAL();
    if (!instance_registered_locked(instance))
    {
        taskEXIT_CRITICAL();
        return DAEMON_ERR_NOT_REGISTERED;
    }
    if ((flags & DAEMON_REPORT_TRANSPORT_VALID) == 0u)
    {
        taskEXIT_CRITICAL();
        return DAEMON_OK;
    }
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t receive_age_ms = elapsed_ms(now_ms, received_ms);
    if (receive_age_ms > INT32_MAX || receive_age_ms > instance->transport_timeout_ms)
    {
        taskEXIT_CRITICAL();
        return DAEMON_ERR_EXPIRED;
    }
    if (instance->has_transport_frame != 0u &&
        elapsed_ms(received_ms, instance->last_transport_rx_ms) > INT32_MAX)
    {
        taskEXIT_CRITICAL();
        return DAEMON_ERR_ARGUMENT;
    }

    // 先按当前时刻结算超时，避免排队到达的旧帧把已过期来源误报为恢复。
    update_state_locked(instance, now_ms);
    instance->last_transport_rx_ms = received_ms;
    instance->has_transport_frame = 1u;
    if ((flags & DAEMON_REPORT_DATA_PRESENT) != 0u)
    {
        instance->last_data_rx_ms = received_ms;
        instance->has_data_sample = 1u;
        instance->data_valid = (uint8_t)((flags & DAEMON_REPORT_DATA_VALID) != 0u);
    }
    update_state_locked(instance, now_ms);
    taskEXIT_CRITICAL();
    return DAEMON_OK;
}

DaemonStatus_t DaemonGetSourceStatus(const DaemonInstance *instance, uint32_t now_ms,
                                     DaemonSourceStatus_t *output)
{
    if (instance == NULL || output == NULL)
        return DAEMON_ERR_ARGUMENT;

    taskENTER_CRITICAL();
    if (!instance_registered_locked(instance))
    {
        taskEXIT_CRITICAL();
        return DAEMON_ERR_NOT_REGISTERED;
    }
    DaemonSourceStatus_t candidate = {0};
    candidate.state = source_state_locked(instance, now_ms);
    candidate.last_transport_rx_ms = instance->last_transport_rx_ms;
    candidate.last_data_rx_ms = instance->last_data_rx_ms;
    candidate.has_transport_frame = instance->has_transport_frame;
    candidate.has_data_sample = instance->has_data_sample;
    candidate.transport_age_ms = instance->has_transport_frame != 0u ?
        elapsed_ms(now_ms, instance->last_transport_rx_ms) : 0u;
    candidate.data_age_ms = instance->has_data_sample != 0u ?
        elapsed_ms(now_ms, instance->last_data_rx_ms) : 0u;
    candidate.transport_deadline_ms = instance->has_transport_frame != 0u ?
        instance->last_transport_rx_ms + instance->transport_timeout_ms :
        instance->registered_ms + instance->startup_grace_ms;
    candidate.data_deadline_ms = instance->has_data_sample != 0u ?
        instance->last_data_rx_ms + instance->data_timeout_ms : 0u;
    candidate.transport_online = (uint8_t)(candidate.state == DAEMON_SOURCE_ONLINE_DATA_VALID ||
                                            candidate.state == DAEMON_SOURCE_ONLINE_DATA_INVALID);
    candidate.data_valid = (uint8_t)(candidate.state == DAEMON_SOURCE_ONLINE_DATA_VALID);
    taskEXIT_CRITICAL();
    *output = candidate;
    return DAEMON_OK;
}

static uint8_t event_is_expired(const DaemonInstance *instance, const DaemonEvent_t *event,
                               uint32_t now_ms)
{
    return (uint8_t)(elapsed_ms(now_ms, event->last_detected_ms) > instance->event_max_age_ms);
}

static uint8_t remove_expired_events_locked(DaemonInstance *instance, uint32_t now_ms)
{
    uint8_t removed = 0u;
    uint8_t i = 0u;
    while (i < instance->event_count)
    {
        const uint8_t slot = (uint8_t)((instance->event_head + i) % DAEMON_EVENT_MAX_DEPTH);
        if (event_is_expired(instance, &instance->events[slot], now_ms) != 0u)
        {
            queue_remove_locked(instance, i);
            instance->expired = saturating_increment(instance->expired);
            global_event_expired = saturating_increment(global_event_expired);
            removed = 1u;
        }
        else
            ++i;
    }
    return removed;
}

static DaemonStatus_t process_source_events(DaemonInstance *instance, uint8_t max_events,
                                            uint32_t now_ms, uint8_t *processed)
{
    uint8_t count = 0u;
    uint8_t expired = 0u;
    if (processed != NULL)
        *processed = 0u;
    if (instance == NULL || max_events == 0u || max_events > DAEMON_EVENT_MAX_DEPTH)
        return DAEMON_ERR_ARGUMENT;

    while (count < max_events)
    {
        DaemonEvent_t event;
        void (*handler)(const DaemonEvent_t *, void *) = NULL;
        void *owner = NULL;
        taskENTER_CRITICAL();
        if (!instance_registered_locked(instance))
        {
            taskEXIT_CRITICAL();
            return DAEMON_ERR_NOT_REGISTERED;
        }
        expired |= remove_expired_events_locked(instance, now_ms);
        if (instance->event_count == 0u)
        {
            taskEXIT_CRITICAL();
            break;
        }
        event = instance->events[instance->event_head];
        instance->event_head = (uint8_t)((instance->event_head + 1u) % DAEMON_EVENT_MAX_DEPTH);
        instance->event_count--;
        instance->dispatched = saturating_increment(instance->dispatched);
        handler = instance->event_handler;
        owner = instance->owner_id;
        taskEXIT_CRITICAL();

        if (handler != NULL)
            handler(&event, owner);
        count++;
    }
    if (processed != NULL)
        *processed = count;
    return count == 0u && expired != 0u ? DAEMON_ERR_EXPIRED : DAEMON_OK;
}

DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer_id, uint8_t max_events,
                                       uint8_t *processed)
{
    if (consumer_id == 0u || max_events == 0u || max_events > DAEMON_EVENT_MAX_DEPTH)
    {
        if (processed != NULL)
            *processed = 0u;
        return DAEMON_ERR_ARGUMENT;
    }
    if (consumer_id == DAEMON_EVENT_TASK_CONSUMER)
    {
        DaemonStats_t stats = {0};
        if (DaemonGetStats(NULL, &stats) != DAEMON_OK || stats.task_started == 0u)
        {
            if (processed != NULL)
                *processed = 0u;
            return DAEMON_ERR_CONSUMER;
        }
        return process_independent_events(max_events, processed);
    }
    if (processed != NULL)
        *processed = 0u;

    DaemonInstance *snapshot[DAEMON_MX_CNT];
    uint8_t count;
    taskENTER_CRITICAL();
    daemon_event_consumer_ran = 1u;
    count = daemon_count;
    memcpy(snapshot, daemon_instances, (size_t)count * sizeof(snapshot[0]));
    taskEXIT_CRITICAL();

    uint8_t total = 0u;
    uint8_t saw_expired = 0u;
    for (uint8_t i = 0u; i < count && total < max_events; ++i)
    {
        DaemonInstance *instance = snapshot[i];
        if (instance == NULL || instance->consumer_mode != DAEMON_CONSUMER_BOUND_TASK ||
            instance->consumer_id != consumer_id)
            continue;
        uint8_t handled = 0u;
        const DaemonStatus_t result = process_source_events(instance,
            (uint8_t)(max_events - total), HAL_GetTick(), &handled);
        if (result == DAEMON_ERR_NOT_REGISTERED)
            continue;
        if (result == DAEMON_ERR_EXPIRED)
            saw_expired = 1u;
        total = (uint8_t)(total + handled);
    }
    if (processed != NULL)
        *processed = total;
    return total == 0u && saw_expired != 0u ? DAEMON_ERR_EXPIRED : DAEMON_OK;
}

static DaemonStatus_t process_independent_events(uint8_t max_events, uint8_t *processed)
{
    if (max_events == 0u || max_events > DAEMON_EVENT_MAX_DEPTH)
        return DAEMON_ERR_ARGUMENT;
    if (processed != NULL)
        *processed = 0u;
    DaemonInstance *snapshot[DAEMON_MX_CNT];
    uint8_t count;
    taskENTER_CRITICAL();
    daemon_event_consumer_ran = 1u;
    count = daemon_count;
    memcpy(snapshot, daemon_instances, (size_t)count * sizeof(snapshot[0]));
    taskEXIT_CRITICAL();

    uint8_t total = 0u;
    uint8_t saw_expired = 0u;
    for (uint8_t i = 0u; i < count && total < max_events; ++i)
    {
        DaemonInstance *instance = snapshot[i];
        if (instance == NULL || instance->consumer_mode != DAEMON_CONSUMER_INDEPENDENT_TASK)
            continue;
        uint8_t handled = 0u;
        const DaemonStatus_t result = process_source_events(instance,
            (uint8_t)(max_events - total), HAL_GetTick(), &handled);
        if (result == DAEMON_ERR_EXPIRED)
            saw_expired = 1u;
        total = (uint8_t)(total + handled);
    }
    if (processed != NULL)
        *processed = total;
    return total == 0u && saw_expired != 0u ? DAEMON_ERR_EXPIRED : DAEMON_OK;
}

uint8_t DaemonHasIndependentSources(void)
{
    uint8_t found = 0u;
    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < daemon_count; ++i)
        if (daemon_instances[i]->consumer_mode == DAEMON_CONSUMER_INDEPENDENT_TASK)
            found = 1u;
    taskEXIT_CRITICAL();
    return found;
}

void DaemonSetEventTaskResult(uint8_t started, uint8_t failed)
{
    taskENTER_CRITICAL();
    event_task_started = (uint8_t)(started != 0u);
    event_task_failed = (uint8_t)(failed != 0u);
    taskEXIT_CRITICAL();
}

DaemonStatus_t DaemonGetStats(const DaemonInstance *instance, DaemonStats_t *output)
{
    if (output == NULL)
        return DAEMON_ERR_ARGUMENT;
    DaemonStats_t candidate = {0};
    taskENTER_CRITICAL();
    if (instance != NULL)
    {
        if (!instance_registered_locked(instance))
        {
            taskEXIT_CRITICAL();
            return DAEMON_ERR_NOT_REGISTERED;
        }
        candidate.enqueued = instance->enqueued;
        candidate.dispatched = instance->dispatched;
        candidate.overflowed = instance->overflowed;
        candidate.expired = instance->expired;
        candidate.queued = instance->event_count;
    }
    candidate.task_started = event_task_started;
    candidate.task_failed = event_task_failed;
    candidate.global_overflowed = global_event_overflow;
    candidate.global_expired = global_event_expired;
    taskEXIT_CRITICAL();
    *output = candidate;
    return DAEMON_OK;
}

void DaemonTask(void)
{
    DaemonInstance *snapshot[DAEMON_MX_CNT];
    uint8_t count;
    const uint32_t now_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    daemon_task_ran = 1u;
    count = daemon_count;
    memcpy(snapshot, daemon_instances, (size_t)count * sizeof(snapshot[0]));
    taskEXIT_CRITICAL();

    for (uint8_t i = 0u; i < count; ++i)
    {
        DaemonInstance *instance = snapshot[i];
        if (instance == NULL)
            continue;
        taskENTER_CRITICAL();
        update_state_locked(instance, now_ms);
        taskEXIT_CRITICAL();
    }
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance != NULL)
        (void)DaemonReportAt(instance, HAL_GetTick(),
            DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    DaemonSourceStatus_t status;
    return (uint8_t)(DaemonGetSourceStatus(instance, HAL_GetTick(), &status) == DAEMON_OK &&
        status.transport_online != 0u);
}
