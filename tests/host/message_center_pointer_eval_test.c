/** @file message_center_pointer_eval_test.c
 * @brief 对比生产消息中心复制接口与带只读租约的快照视图模型。
 * @note 仅用于WSL主机评估；视图模型不是生产接口，主机耗时不代表H723执行时间。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#define _GNU_SOURCE

#include "test_check.h"
#include "message_center.h"
#include "robot_topics.h"
#include "FreeRTOS.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define EVAL_BATCH_COUNT 20u
#define EVAL_CYCLES_PER_BATCH 10000u
#define EVAL_WARMUP_CYCLES 1000u
#define EVAL_VIEW_SLOTS_PER_TOPIC (MESSAGE_CENTER_MAX_SUBSCRIBERS + 2u)
#define EVAL_NO_SLOT UINT8_MAX

typedef struct
{
    uint8_t *bytes;
    uint16_t references;
    uint32_t generation;
    MessageFreshness_t freshness;
} PointerSlot_t;

typedef struct
{
    uint8_t active;
    uint8_t queue_slot;
    uint8_t lease_active;
    uint8_t lease_slot;
    uint32_t lease_generation;
} PointerSubscriber_t;

typedef struct
{
    PointerSlot_t slots[EVAL_VIEW_SLOTS_PER_TOPIC];
    PointerSubscriber_t subscribers[MESSAGE_CENTER_MAX_SUBSCRIBERS];
    uint8_t subscriber_count;
    uint8_t slot_count;
    uint8_t active;
    uint16_t payload_size;
    uint32_t next_generation;
    MessageTopicDescriptor_t descriptor;
} PointerTopic_t;

typedef struct
{
    const uint8_t *data;
    uint32_t generation;
    MessageFreshness_t freshness;
    uint8_t slot;
    uint8_t subscriber;
} PointerView_t;

typedef struct
{
    MessageTopicDiagnostic_t diagnostic;
    MessagePublisher_t *publisher;
    MessageSubscriber_t *subscribers[MESSAGE_CENTER_MAX_SUBSCRIBERS];
    uint8_t subscriber_count;
    PointerTopic_t view_topic;
} TopicCase_t;

typedef struct
{
    uint64_t mean_ns;
    uint64_t p50_batch_ns;
    uint64_t p95_batch_ns;
    uint64_t max_batch_ns;
} TimingResult_t;

static TopicCase_t topic_cases[MESSAGE_CENTER_MAX_TOPICS];
static uint8_t pointer_payload_storage[MESSAGE_CENTER_MAX_PAYLOAD_STORAGE_BYTES];
static size_t pointer_payload_storage_used;
static uint64_t copied_bytes;
static uint8_t count_copies;
static uint32_t test_tick_ms = 100u;
static pthread_mutex_t critical_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct
{
    PointerTopic_t *topic;
    uint32_t first_sequence;
    atomic_uint done;
    atomic_uint failed;
} ViewConcurrency_t;

uint32_t HAL_GetTick(void)
{
    return test_tick_ms;
}

void HostCriticalEnter(void)
{
    (void)pthread_mutex_lock(&critical_mutex);
}

void HostCriticalExit(void)
{
    (void)pthread_mutex_unlock(&critical_mutex);
}

void *__real_memcpy(void *destination, const void *source, size_t size);

void *__wrap_memcpy(void *destination, const void *source, size_t size)
{
    if (count_copies != 0u)
        copied_bytes += size;
    return __real_memcpy(destination, source, size);
}

static int ThreadCpuNs(uint64_t *output)
{
    struct timespec now;
    if (output == NULL || clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now) != 0)
        return 0;

    *output = (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
    return 1;
}

static int CompareU64(const void *left, const void *right)
{
    const uint64_t a = *(const uint64_t *)left;
    const uint64_t b = *(const uint64_t *)right;
    return (a > b) - (a < b);
}

static uint8_t RegisterRobotTopics(void)
{
    if (RobotTopicsInit() == 0u)
        return 0u;

    uint8_t topic_count = 0u;
    if (MessageGetDiagnosticTopicCount(&topic_count) != MESSAGE_OK ||
        topic_count == 0u || topic_count > MESSAGE_CENTER_MAX_TOPICS)
        return 0u;

    for (uint8_t topic_index = 0u; topic_index < topic_count; ++topic_index)
    {
        TopicCase_t *topic = &topic_cases[topic_index];
        if (MessageGetTopicDiagnostic(topic_index, &topic->diagnostic) != MESSAGE_OK)
            return 0u;

        const uint32_t publisher_mask = topic->diagnostic.descriptor.publisher_mask;
        const uint32_t subscriber_mask = topic->diagnostic.descriptor.subscriber_mask;
        uint8_t publisher_count = 0u;
        for (uint8_t endpoint = 1u; endpoint <= 31u; ++endpoint)
        {
            const uint32_t endpoint_bit = UINT32_C(1) << (endpoint - 1u);
            if ((publisher_mask & endpoint_bit) != 0u)
            {
                topic->publisher = MessagePublisherRegister(
                    topic->diagnostic.descriptor.topic_id, endpoint);
                if (topic->publisher == NULL)
                    return 0u;
                ++publisher_count;
            }
            if ((subscriber_mask & endpoint_bit) != 0u)
            {
                if (topic->subscriber_count >= MESSAGE_CENTER_MAX_SUBSCRIBERS)
                    return 0u;
                MessageSubscriber_t *subscriber = MessageSubscriberRegister(
                    topic->diagnostic.descriptor.topic_id, endpoint);
                if (subscriber == NULL)
                    return 0u;
                topic->subscribers[topic->subscriber_count++] = subscriber;
            }
        }
        if (publisher_count != 1u || topic->publisher == NULL ||
            topic->subscriber_count != 1u ||
            topic->diagnostic.descriptor.delivery != MESSAGE_DELIVERY_SNAPSHOT ||
            topic->diagnostic.descriptor.capacity != 1u ||
            topic->diagnostic.descriptor.payload_size > MESSAGE_CENTER_MAX_PAYLOAD_BYTES)
            return 0u;
    }
    return topic_count;
}

static uint8_t PointerTopicInit(PointerTopic_t *topic,
                                const MessageTopicDescriptor_t *descriptor,
                                uint8_t subscriber_count)
{
    if (topic == NULL || descriptor == NULL || descriptor->payload_size == 0u ||
        descriptor->payload_size > MESSAGE_CENTER_MAX_PAYLOAD_BYTES || subscriber_count == 0u ||
        subscriber_count > MESSAGE_CENTER_MAX_SUBSCRIBERS)
        return 0u;

    const uint8_t slot_count = (uint8_t)(subscriber_count + 2u);
    const size_t reserve_bytes = (size_t)slot_count * descriptor->payload_size;
    if (reserve_bytes > sizeof(pointer_payload_storage) - pointer_payload_storage_used)
        return 0u;

    memset(topic, 0, sizeof(*topic));
    topic->subscriber_count = subscriber_count;
    topic->slot_count = slot_count;
    topic->payload_size = descriptor->payload_size;
    topic->descriptor = *descriptor;
    topic->active = 1u;
    const size_t storage_offset = pointer_payload_storage_used;
    for (uint8_t i = 0u; i < subscriber_count; ++i)
    {
        topic->subscribers[i].active = 1u;
        topic->subscribers[i].queue_slot = EVAL_NO_SLOT;
        topic->subscribers[i].lease_slot = EVAL_NO_SLOT;
    }
    for (uint8_t i = 0u; i < slot_count; ++i)
    {
        topic->slots[i].bytes = &pointer_payload_storage[
            storage_offset + (size_t)i * descriptor->payload_size];
        topic->slots[i].references = 0u;
        topic->slots[i].generation = 0u;
        memset(&topic->slots[i].freshness, 0, sizeof(topic->slots[i].freshness));
    }

    pointer_payload_storage_used += reserve_bytes;
    return 1u;
}

static uint8_t PointerTopicPublish(PointerTopic_t *topic, const void *data,
                                   uint32_t source_time_ms, uint32_t received_time_ms,
                                   uint32_t published_time_ms,
                                   MessageClockDomain_t source_clock)
{
    if (topic == NULL || topic->active == 0u || data == NULL ||
        source_clock != topic->descriptor.source_clock)
        return 0u;

    taskENTER_CRITICAL();
    uint8_t candidate = EVAL_NO_SLOT;
    for (uint8_t i = 0u; i < topic->slot_count; ++i)
        if (topic->slots[i].references == 0u)
        {
            candidate = i;
            break;
        }
    if (candidate == EVAL_NO_SLOT)
    {
        // 没有空闲数据槽时保留当前内容，避免改写仍被读者持有的字节。
        taskEXIT_CRITICAL();
        return 0u;
    }

    PointerSlot_t *new_slot = &topic->slots[candidate];
    memcpy(new_slot->bytes, data, topic->payload_size);
    ++topic->next_generation;
    if (topic->next_generation == 0u)
        topic->next_generation = 1u;
    new_slot->generation = topic->next_generation;
    new_slot->freshness.valid = 1u;
    new_slot->freshness.sequence = topic->next_generation;
    new_slot->freshness.source_time_ms = source_time_ms;
    new_slot->freshness.received_time_ms = received_time_ms;
    new_slot->freshness.published_time_ms = published_time_ms;
    new_slot->freshness.source_clock = (uint8_t)source_clock;

    for (uint8_t i = 0u; i < topic->subscriber_count; ++i)
    {
        PointerSubscriber_t *subscriber = &topic->subscribers[i];
        if (subscriber->active == 0u)
            continue;
        if (subscriber->queue_slot != EVAL_NO_SLOT)
            --topic->slots[subscriber->queue_slot].references;
        subscriber->queue_slot = candidate;
        ++new_slot->references;
    }
    taskEXIT_CRITICAL();
    return 1u;
}

static uint8_t PointerViewAcquire(PointerTopic_t *topic, uint8_t subscriber_index,
                                  uint32_t now_ms, PointerView_t *output)
{
    if (topic == NULL || output == NULL || topic->active == 0u ||
        subscriber_index >= topic->subscriber_count)
        return 0u;

    taskENTER_CRITICAL();
    PointerSubscriber_t *subscriber = &topic->subscribers[subscriber_index];
    if (subscriber->active == 0u || subscriber->lease_active != 0u ||
        subscriber->queue_slot == EVAL_NO_SLOT)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }

    const uint8_t slot_index = subscriber->queue_slot;
    PointerSlot_t *slot = &topic->slots[slot_index];
    const uint32_t age_origin = slot->freshness.source_clock == MESSAGE_CLOCK_REMOTE_RAW ?
                                slot->freshness.received_time_ms :
                                slot->freshness.source_time_ms;
    if (slot->freshness.valid == 0u ||
        (uint32_t)(now_ms - age_origin) > topic->descriptor.max_age_ms)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }
    ++slot->references;
    subscriber->lease_active = 1u;
    subscriber->lease_slot = slot_index;
    subscriber->lease_generation = slot->generation;
    output->data = slot->bytes;
    output->generation = slot->generation;
    output->freshness = slot->freshness;
    output->slot = slot_index;
    output->subscriber = subscriber_index;
    taskEXIT_CRITICAL();
    return 1u;
}

static uint8_t PointerViewRelease(PointerTopic_t *topic, PointerView_t *view)
{
    if (topic == NULL || view == NULL || topic->active == 0u ||
        view->subscriber >= topic->subscriber_count || view->slot >= topic->slot_count)
        return 0u;

    taskENTER_CRITICAL();
    PointerSubscriber_t *subscriber = &topic->subscribers[view->subscriber];
    PointerSlot_t *slot = &topic->slots[view->slot];
    if (subscriber->lease_active == 0u || subscriber->lease_slot != view->slot ||
        subscriber->lease_generation != view->generation ||
        slot->generation != view->generation || slot->references == 0u)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }

    --slot->references;
    subscriber->lease_active = 0u;
    subscriber->lease_slot = EVAL_NO_SLOT;
    subscriber->lease_generation = 0u;
    memset(view, 0, sizeof(*view));
    taskEXIT_CRITICAL();
    return 1u;
}

static uint8_t FillSample(uint8_t *output, size_t length, uint32_t sequence)
{
    if (output == NULL || length < 2u)
        return 0u;
    output[0] = (uint8_t)sequence;
    output[length - 1u] = (uint8_t)(sequence >> 8u);
    return 1u;
}

static uint8_t SampleMatches(const uint8_t *data, size_t length, uint32_t sequence)
{
    return (uint8_t)(data != NULL && length >= 2u && data[0] == (uint8_t)sequence &&
                     data[length - 1u] == (uint8_t)(sequence >> 8u));
}

static uint8_t PrepareTopic(TopicCase_t *topic, uint8_t *source, uint32_t sequence)
{
    const MessageTopicDescriptor_t *descriptor = &topic->diagnostic.descriptor;
    const size_t size = descriptor->payload_size;
    if (!FillSample(source, size, sequence) ||
        MessagePublishAt(topic->publisher, source, test_tick_ms, test_tick_ms,
                         test_tick_ms, descriptor->source_clock) != MESSAGE_OK ||
        !PointerTopicPublish(&topic->view_topic, source, test_tick_ms, test_tick_ms,
                             test_tick_ms, descriptor->source_clock))
        return 0u;
    memset(source, 0xEE, size); /* 成功发布后，调用方可立刻复用源缓冲。 */
    return 1u;
}

static uint8_t RunCopiedRead(const TopicCase_t *topic, uint8_t *output,
                             uint32_t sequence)
{
    const size_t size = topic->diagnostic.descriptor.payload_size;
    if (MessageReadSnapshot(topic->subscribers[0], output, size, test_tick_ms, NULL) != MESSAGE_OK)
        return 0u;
    return SampleMatches(output, size, sequence);
}

static uint8_t RunViewRead(TopicCase_t *topic, uint32_t sequence)
{
    PointerView_t view = {0};
    const size_t size = topic->diagnostic.descriptor.payload_size;
    if (!PointerViewAcquire(&topic->view_topic, 0u, test_tick_ms, &view))
        return 0u;
    const uint8_t matches = SampleMatches(view.data, size, sequence);
    const uint8_t released = PointerViewRelease(&topic->view_topic, &view);
    return (uint8_t)(matches && released);
}

static uint8_t MeasureTopic(TopicCase_t *topic, uint32_t topic_index,
                            TimingResult_t *copied_result, TimingResult_t *view_result,
                            uint64_t *copied_bytes_per_cycle,
                            uint64_t *view_bytes_per_cycle)
{
    static uint64_t samples[EVAL_BATCH_COUNT];
    static uint8_t source[MESSAGE_CENTER_MAX_PAYLOAD_BYTES];
    static uint8_t output[MESSAGE_CENTER_MAX_PAYLOAD_BYTES];
    const uint64_t total_cycles = (uint64_t)EVAL_BATCH_COUNT * EVAL_CYCLES_PER_BATCH;
    const uint32_t sequence = UINT32_C(0xA500) + topic_index;

    if (!PrepareTopic(topic, source, sequence))
        return 0u;
    for (uint32_t i = 0u; i < EVAL_WARMUP_CYCLES; ++i)
        if (!RunCopiedRead(topic, output, sequence))
            return 0u;

    copied_bytes = 0u;
    uint64_t total_elapsed_ns = 0u;
    for (size_t batch = 0u; batch < EVAL_BATCH_COUNT; ++batch)
    {
        uint64_t start_ns;
        uint64_t end_ns;
        if (!ThreadCpuNs(&start_ns))
            return 0u;
        count_copies = 1u;
        for (uint32_t cycle = 0u; cycle < EVAL_CYCLES_PER_BATCH; ++cycle)
            if (!RunCopiedRead(topic, output, sequence))
            {
                count_copies = 0u;
                return 0u;
            }
        count_copies = 0u;
        if (!ThreadCpuNs(&end_ns))
            return 0u;
        const uint64_t elapsed_ns = end_ns - start_ns;
        total_elapsed_ns += elapsed_ns;
        samples[batch] = elapsed_ns / EVAL_CYCLES_PER_BATCH;
    }
    *copied_bytes_per_cycle = copied_bytes / total_cycles;
    if (*copied_bytes_per_cycle != topic->diagnostic.descriptor.payload_size)
        return 0u;
    qsort(samples, EVAL_BATCH_COUNT, sizeof(samples[0]), CompareU64);
    copied_result->mean_ns = total_elapsed_ns / total_cycles;
    copied_result->p50_batch_ns = samples[(EVAL_BATCH_COUNT / 2u) - 1u];
    copied_result->p95_batch_ns = samples[((EVAL_BATCH_COUNT * 95u + 99u) / 100u) - 1u];
    copied_result->max_batch_ns = samples[EVAL_BATCH_COUNT - 1u];

    for (uint32_t i = 0u; i < EVAL_WARMUP_CYCLES; ++i)
        if (!RunViewRead(topic, sequence))
            return 0u;

    copied_bytes = 0u;
    total_elapsed_ns = 0u;
    for (size_t batch = 0u; batch < EVAL_BATCH_COUNT; ++batch)
    {
        uint64_t start_ns;
        uint64_t end_ns;
        if (!ThreadCpuNs(&start_ns))
            return 0u;
        count_copies = 1u;
        for (uint32_t cycle = 0u; cycle < EVAL_CYCLES_PER_BATCH; ++cycle)
            if (!RunViewRead(topic, sequence))
            {
                count_copies = 0u;
                return 0u;
            }
        count_copies = 0u;
        if (!ThreadCpuNs(&end_ns))
            return 0u;
        const uint64_t elapsed_ns = end_ns - start_ns;
        total_elapsed_ns += elapsed_ns;
        samples[batch] = elapsed_ns / EVAL_CYCLES_PER_BATCH;
    }
    *view_bytes_per_cycle = copied_bytes / total_cycles;
    if (*view_bytes_per_cycle != 0u)
        return 0u;
    qsort(samples, EVAL_BATCH_COUNT, sizeof(samples[0]), CompareU64);
    view_result->mean_ns = total_elapsed_ns / total_cycles;
    view_result->p50_batch_ns = samples[(EVAL_BATCH_COUNT / 2u) - 1u];
    view_result->p95_batch_ns = samples[((EVAL_BATCH_COUNT * 95u + 99u) / 100u) - 1u];
    view_result->max_batch_ns = samples[EVAL_BATCH_COUNT - 1u];
    return 1u;
}

static uint8_t CheckViewLifetime(TopicCase_t *topic)
{
    uint8_t source[MESSAGE_CENTER_MAX_PAYLOAD_BYTES] = {0};
    PointerView_t old_view = {0};
    PointerView_t current_view = {0};
    const size_t size = topic->diagnostic.descriptor.payload_size;
    if (!FillSample(source, size, 0x1234u) ||
        !PointerTopicPublish(&topic->view_topic, source, test_tick_ms, test_tick_ms,
                             test_tick_ms,
                             (MessageClockDomain_t)topic->diagnostic.descriptor.source_clock) ||
        !PointerViewAcquire(&topic->view_topic, 0u, test_tick_ms, &old_view))
        return 0u;
    memset(source, 0xEE, size);
    if (!FillSample(source, size, 0x5678u) ||
        !PointerTopicPublish(&topic->view_topic, source, test_tick_ms, test_tick_ms,
                             test_tick_ms,
                             (MessageClockDomain_t)topic->diagnostic.descriptor.source_clock) ||
        !SampleMatches(old_view.data, size, 0x1234u))
        return 0u;
    if (PointerViewAcquire(&topic->view_topic, 0u, test_tick_ms, &current_view) != 0u)
        return 0u; /* 同一订阅者持有租约时，不能再取得第二份视图。 */
    if (!PointerViewRelease(&topic->view_topic, &old_view) ||
        !PointerViewAcquire(&topic->view_topic, 0u, test_tick_ms, &current_view) ||
        !SampleMatches(current_view.data, size, 0x5678u))
        return 0u;
    if (!FillSample(source, size, 0x9ABCu) ||
        !PointerTopicPublish(&topic->view_topic, source, test_tick_ms, test_tick_ms,
                             test_tick_ms,
                             (MessageClockDomain_t)topic->diagnostic.descriptor.source_clock) ||
        !SampleMatches(current_view.data, size, 0x5678u))
        return 0u;
    return PointerViewRelease(&topic->view_topic, &current_view);
}

static void *PublishViewRecords(void *argument)
{
    ViewConcurrency_t *test = argument;
    uint8_t source[MESSAGE_CENTER_MAX_PAYLOAD_BYTES];
    for (uint32_t offset = 0u; offset < 100000u; ++offset)
    {
        const uint32_t sequence = test->first_sequence + offset;
        if (!FillSample(source, test->topic->payload_size, sequence) ||
            !PointerTopicPublish(test->topic, source, test_tick_ms, test_tick_ms,
                                 test_tick_ms,
                                 (MessageClockDomain_t)test->topic->descriptor.source_clock))
        {
            atomic_store_explicit(&test->failed, 1u, memory_order_release);
            break;
        }
    }
    atomic_store_explicit(&test->done, 1u, memory_order_release);
    return NULL;
}

static void *ReadViewRecords(void *argument)
{
    ViewConcurrency_t *test = argument;
    do
    {
        PointerView_t view = {0};
        if (!PointerViewAcquire(test->topic, 0u, test_tick_ms, &view))
            continue;
        const uint8_t matches = SampleMatches(
            view.data, test->topic->payload_size, view.generation);
        const uint8_t released = PointerViewRelease(test->topic, &view);
        if (!matches || !released)
        {
            atomic_store_explicit(&test->failed, 1u, memory_order_release);
            break;
        }
    } while (atomic_load_explicit(&test->done, memory_order_acquire) == 0u);
    return NULL;
}

static uint8_t CheckViewConcurrency(TopicCase_t *topic)
{
    // 上一个场景留下任意样本；先发布与编号一致的数据，再启动并发读写。
    uint8_t initial_data[MESSAGE_CENTER_MAX_PAYLOAD_BYTES];
    const uint32_t initial_generation = topic->view_topic.next_generation + 1u;
    if (!FillSample(initial_data, topic->view_topic.payload_size, initial_generation) ||
        !PointerTopicPublish(&topic->view_topic, initial_data, test_tick_ms, test_tick_ms,
                             test_tick_ms,
                             (MessageClockDomain_t)topic->view_topic.descriptor.source_clock))
        return 0u;
    // 并发读者可能先运行；启动前已有的数据也必须与其编号一致。
    PointerView_t initial = {0};
    if (!PointerViewAcquire(&topic->view_topic, 0u, test_tick_ms, &initial))
        return 0u;
    const uint8_t initial_matches = SampleMatches(initial.data, topic->view_topic.payload_size,
                                                   initial.generation);
    const uint8_t initial_released = PointerViewRelease(&topic->view_topic, &initial);
    if (!initial_matches || !initial_released)
        return 0u;
    ViewConcurrency_t test = {
        .topic = &topic->view_topic,
        .first_sequence = topic->view_topic.next_generation + 1u,
    };
    atomic_init(&test.done, 0u);
    atomic_init(&test.failed, 0u);
    pthread_t publisher_thread;
    pthread_t reader_thread;
    if (pthread_create(&publisher_thread, NULL, PublishViewRecords, &test) != 0)
        return 0u;
    if (pthread_create(&reader_thread, NULL, ReadViewRecords, &test) != 0)
    {
        atomic_store_explicit(&test.done, 1u, memory_order_release);
        (void)pthread_join(publisher_thread, NULL);
        return 0u;
    }
    const int publisher_joined = pthread_join(publisher_thread, NULL) == 0;
    const int reader_joined = pthread_join(reader_thread, NULL) == 0;
    return (uint8_t)(publisher_joined && reader_joined &&
                     atomic_load_explicit(&test.failed, memory_order_acquire) == 0u);
}

static uint8_t CheckMultipleLeases(void)
{
    PointerTopic_t topic;
    PointerView_t retained[MESSAGE_CENTER_MAX_SUBSCRIBERS] = {{0}};
    uint8_t source[MESSAGE_CENTER_MAX_PAYLOAD_BYTES] = {0};
    const uint8_t subscriber_count = 3u;
    static const MessageTopicDescriptor_t descriptor = {
        .topic_id = 900u,
        .payload_size = 8u,
        .max_age_ms = 100u,
        .capacity = 1u,
        .delivery = MESSAGE_DELIVERY_SNAPSHOT,
        .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 7u,
        .diagnostics_enabled = 1u,
    };
    if (!PointerTopicInit(&topic, &descriptor, subscriber_count))
        return 0u;

    for (uint8_t subscriber = 0u; subscriber < subscriber_count; ++subscriber)
    {
        const uint32_t sequence = (uint32_t)subscriber + 1u;
        if (!FillSample(source, topic.payload_size, sequence) ||
            !PointerTopicPublish(&topic, source, test_tick_ms, test_tick_ms,
                                 test_tick_ms, MESSAGE_CLOCK_LOCAL) ||
            !PointerViewAcquire(&topic, subscriber, test_tick_ms,
                                &retained[subscriber]))
            return 0u;
    }

    // 多个读取者各自保留旧版本时，发布者仍能更新最新队列内容。
    for (uint32_t sequence = 10u; sequence < 100u; ++sequence)
    {
        if (!FillSample(source, topic.payload_size, sequence) ||
            !PointerTopicPublish(&topic, source, test_tick_ms, test_tick_ms,
                                 test_tick_ms, MESSAGE_CLOCK_LOCAL))
            return 0u;
        for (uint8_t subscriber = 0u; subscriber < subscriber_count; ++subscriber)
            if (!SampleMatches(retained[subscriber].data, topic.payload_size,
                               (uint32_t)subscriber + 1u))
                return 0u;
    }

    for (uint8_t subscriber = 0u; subscriber < subscriber_count; ++subscriber)
    {
        PointerView_t latest = {0};
        if (!PointerViewRelease(&topic, &retained[subscriber]) ||
            !PointerViewAcquire(&topic, subscriber, test_tick_ms, &latest) ||
            !SampleMatches(latest.data, topic.payload_size, 99u) ||
            !PointerViewRelease(&topic, &latest))
            return 0u;
    }
    return 1u;
}

int main(void)
{
    const uint8_t topic_count = RegisterRobotTopics();
    TEST_CHECK(topic_count != 0u);

    size_t copied_pool_bytes = 0u;
    size_t topic_view_pool_bytes = 0u;
    for (uint8_t i = 0u; i < topic_count; ++i)
    {
        TopicCase_t *topic = &topic_cases[i];
        copied_pool_bytes += (size_t)topic->diagnostic.descriptor.payload_size *
                             topic->diagnostic.descriptor.capacity * topic->subscriber_count;
        TEST_CHECK(PointerTopicInit(&topic->view_topic,
                                    &topic->diagnostic.descriptor,
                                    topic->subscriber_count));
        topic_view_pool_bytes += (size_t)topic->view_topic.payload_size *
                                 topic->view_topic.slot_count;
    }

    TEST_CHECK(CheckViewLifetime(&topic_cases[0]));
    TEST_CHECK(CheckMultipleLeases());
    TEST_CHECK(pointer_payload_storage_used <= sizeof(pointer_payload_storage));
    TEST_CHECK(CheckViewConcurrency(&topic_cases[0]));

    printf("{\"scenario\":\"robot_topics_snapshot_copy_vs_leased_view\","
           "\"topics\":%u,\"subscriber_queue_bytes\":%zu,"
           "\"topic_view_pool_bytes\":%zu,\"multi_lease_fixture_bytes\":%zu,"
           "\"view_pool_limit\":%zu}\n",
           topic_count, copied_pool_bytes, topic_view_pool_bytes,
           pointer_payload_storage_used - topic_view_pool_bytes,
           sizeof(pointer_payload_storage));
    for (uint8_t i = 0u; i < topic_count; ++i)
    {
        TopicCase_t *topic = &topic_cases[i];
        TimingResult_t copied = {0};
        TimingResult_t view = {0};
        uint64_t copied_bytes_per_cycle = 0u;
        uint64_t view_bytes_per_cycle = 0u;
        TEST_CHECK(MeasureTopic(topic, i, &copied, &view,
                                &copied_bytes_per_cycle, &view_bytes_per_cycle));
        printf("{\"topic_id\":%u,\"payload_bytes\":%u,\"subscribers\":%u,"
               "\"copy_bytes_per_read\":%llu,\"view_bytes_per_read\":%llu,"
               "\"copy_mean_ns\":%llu,\"copy_p50_batch_ns\":%llu,"
               "\"copy_p95_batch_ns\":%llu,\"view_mean_ns\":%llu,"
               "\"view_p50_batch_ns\":%llu,\"view_p95_batch_ns\":%llu}\n",
               topic->diagnostic.descriptor.topic_id,
               topic->diagnostic.descriptor.payload_size,
               topic->subscriber_count,
               (unsigned long long)copied_bytes_per_cycle,
               (unsigned long long)view_bytes_per_cycle,
               (unsigned long long)copied.mean_ns,
               (unsigned long long)copied.p50_batch_ns,
               (unsigned long long)copied.p95_batch_ns,
               (unsigned long long)view.mean_ns,
               (unsigned long long)view.p50_batch_ns,
               (unsigned long long)view.p95_batch_ns);
    }
    return 0;
}
