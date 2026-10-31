/** @file message_center.c
 * @brief 管理启动时固定的类型话题、快照副本和事件FIFO。
 * @note 所有对象使用静态存储；锁只保护有限长度的目录和消息复制。
 * @copyright Copyright (c) 2022
 */
#include "message_center.h"

#include <limits.h>
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define MESSAGE_ENDPOINT_MIN 1u
#define MESSAGE_ENDPOINT_MAX 31u

typedef struct
{
    MessageFreshness_t freshness;
    uint8_t expiry_counted;
} MessageSlot_t;

typedef struct
{
    uint8_t active;
    MessageTopicId_t topic_id;
    MessageEndpointId_t endpoint_id;
} MessagePublisherStorage_t;

struct MessagePublisher
{
    uint8_t storage_index;
};

struct MessageSubscriber
{
    uint8_t active;
    uint8_t topic_index;
    MessageEndpointId_t endpoint_id;
    uint8_t head;
    uint8_t count;
    uint8_t storage_index;
    uint16_t payload_offset;
};

typedef struct
{
    uint8_t active;
    MessageTopicDescriptor_t descriptor;
    uint32_t sequence;
    MessageTopicStats_t stats;
} MessageTopicStorage_t;

static MessageTopicStorage_t topics[MESSAGE_CENTER_MAX_TOPICS];
static MessagePublisherStorage_t publisher_storage[MESSAGE_CENTER_MAX_SUBSCRIBERS];
static MessagePublisher_t publishers[MESSAGE_CENTER_MAX_SUBSCRIBERS];
static struct MessageSubscriber subscribers[MESSAGE_CENTER_MAX_SUBSCRIBERS];
static MessageSlot_t message_slots[MESSAGE_CENTER_MAX_SUBSCRIBERS][MESSAGE_CENTER_MAX_QUEUE_DEPTH];
static uint8_t payload_storage[MESSAGE_CENTER_MAX_PAYLOAD_STORAGE_BYTES];
static size_t payload_storage_used;
static size_t topic_count;
static uint8_t directory_ready;

static MessageSlot_t *GetSlotMetadata(const struct MessageSubscriber *subscriber,
                                      uint8_t slot_index);
static uint8_t *GetSlotData(const struct MessageSubscriber *subscriber,
                            const MessageTopicDescriptor_t *descriptor,
                            uint8_t slot_index);

static uint8_t IsEndpointValid(MessageEndpointId_t endpoint_id)
{
    return (uint8_t)(endpoint_id >= MESSAGE_ENDPOINT_MIN &&
                     endpoint_id <= MESSAGE_ENDPOINT_MAX);
}

static void CountOnce(uint32_t *counter)
{
    // 到达上限后保持原值，避免无符号回绕让累计次数变小。
    if (*counter != UINT32_MAX)
        ++(*counter);
}

static uint8_t IsDescriptorEqual(const MessageTopicDescriptor_t *left,
                                 const MessageTopicDescriptor_t *right)
{
    return (uint8_t)(left->topic_id == right->topic_id &&
                     left->payload_size == right->payload_size &&
                     left->max_age_ms == right->max_age_ms &&
                     left->capacity == right->capacity &&
                     left->delivery == right->delivery &&
                     left->overflow == right->overflow &&
                     left->source_clock == right->source_clock &&
                     left->publisher_mask == right->publisher_mask &&
                     left->subscriber_mask == right->subscriber_mask &&
                     left->diagnostics_enabled == right->diagnostics_enabled);
}

static int FindTopicIndex(MessageTopicId_t topic_id)
{
    for (size_t i = 0u; i < topic_count; ++i)
        if (topics[i].active && topics[i].descriptor.topic_id == topic_id)
            return (int)i;
    return -1;
}

static uint8_t IsDescriptorValid(const MessageTopicDescriptor_t *descriptor)
{
    if (descriptor->topic_id == 0u || descriptor->payload_size == 0u ||
        descriptor->payload_size > MESSAGE_CENTER_MAX_PAYLOAD_BYTES ||
        descriptor->max_age_ms == 0u || descriptor->max_age_ms > (uint32_t)INT32_MAX ||
        descriptor->capacity == 0u ||
        descriptor->capacity > MESSAGE_CENTER_MAX_QUEUE_DEPTH ||
        descriptor->publisher_mask == 0u || descriptor->subscriber_mask == 0u)
        return 0u;
    if (descriptor->diagnostics_enabled > 1u)
        return 0u;

    if (descriptor->source_clock != MESSAGE_CLOCK_LOCAL &&
        descriptor->source_clock != MESSAGE_CLOCK_REMOTE_RAW &&
        descriptor->source_clock != MESSAGE_CLOCK_LOCAL_MAPPED)
        return 0u;

    if (descriptor->delivery == MESSAGE_DELIVERY_SNAPSHOT)
        return (uint8_t)(descriptor->capacity == 1u &&
                         descriptor->overflow == MESSAGE_OVERFLOW_REPLACE_OLDEST);

    if (descriptor->delivery == MESSAGE_DELIVERY_EVENT_FIFO)
        return (uint8_t)(descriptor->overflow == MESSAGE_OVERFLOW_REPLACE_OLDEST ||
                         descriptor->overflow == MESSAGE_OVERFLOW_REJECT_NEWEST);

    return 0u;
}

MessageStatus_t MessageCenterInit(const MessageTopicDescriptor_t *descriptors, size_t count)
{
    if (descriptors == NULL || count == 0u || count > MESSAGE_CENTER_MAX_TOPICS)
        return MESSAGE_ERR_ARGUMENT;

    for (size_t i = 0u; i < count; ++i)
    {
        if (!IsDescriptorValid(&descriptors[i]))
            return MESSAGE_ERR_DIRECTORY;
        for (size_t j = 0u; j < i; ++j)
            if (descriptors[i].topic_id == descriptors[j].topic_id)
                return MESSAGE_ERR_DIRECTORY;
    }

    taskENTER_CRITICAL();
    if (directory_ready)
    {
        uint8_t same = (uint8_t)(count == topic_count);
        for (size_t i = 0u; same && i < count; ++i)
        {
            const int index = FindTopicIndex(descriptors[i].topic_id);
            same = (uint8_t)(index >= 0 &&
                IsDescriptorEqual(&topics[index].descriptor, &descriptors[i]));
        }
        taskEXIT_CRITICAL();
        return same ? MESSAGE_OK : MESSAGE_ERR_DIRECTORY;
    }

    memset(topics, 0, sizeof(topics));
    memset(publisher_storage, 0, sizeof(publisher_storage));
    memset(publishers, 0, sizeof(publishers));
    memset(subscribers, 0, sizeof(subscribers));
    memset(message_slots, 0, sizeof(message_slots));
    memset(payload_storage, 0, sizeof(payload_storage));
    payload_storage_used = 0u;
    topic_count = count;
    for (size_t i = 0u; i < count; ++i)
    {
        topics[i].descriptor = descriptors[i];
        topics[i].active = 1u;
    }
    directory_ready = 1u;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessagePublisher_t *MessagePublisherRegister(MessageTopicId_t topic_id,
                                             MessageEndpointId_t endpoint_id)
{
    if (!directory_ready || !IsEndpointValid(endpoint_id))
        return NULL;
    const int topic_index = FindTopicIndex(topic_id);
    if (topic_index < 0 ||
        (topics[topic_index].descriptor.publisher_mask & (1u << (endpoint_id - 1u))) == 0u)
        return NULL;

    taskENTER_CRITICAL();
    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
    {
        if (publisher_storage[i].active && publisher_storage[i].topic_id == topic_id &&
            publisher_storage[i].endpoint_id == endpoint_id)
        {
            taskEXIT_CRITICAL();
            return &publishers[i];
        }
    }
    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
    {
        if (!publisher_storage[i].active)
        {
            publisher_storage[i].active = 1u;
            publisher_storage[i].topic_id = topic_id;
            publisher_storage[i].endpoint_id = endpoint_id;
            publishers[i].storage_index = (uint8_t)i;
            taskEXIT_CRITICAL();
            return &publishers[i];
        }
    }
    taskEXIT_CRITICAL();
    return NULL;
}

MessageSubscriber_t *MessageSubscriberRegister(MessageTopicId_t topic_id,
                                               MessageEndpointId_t endpoint_id)
{
    if (!directory_ready || !IsEndpointValid(endpoint_id))
        return NULL;
    const int topic_index = FindTopicIndex(topic_id);
    if (topic_index < 0 ||
        (topics[topic_index].descriptor.subscriber_mask & (1u << (endpoint_id - 1u))) == 0u)
        return NULL;

    taskENTER_CRITICAL();
    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
        if (subscribers[i].active && subscribers[i].topic_index == (uint8_t)topic_index &&
            subscribers[i].endpoint_id == endpoint_id)
        {
            taskEXIT_CRITICAL();
            return &subscribers[i];
        }

    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
        if (!subscribers[i].active)
        {
            const MessageTopicDescriptor_t *descriptor = &topics[topic_index].descriptor;
            const size_t payload_bytes =
                (size_t)descriptor->payload_size * descriptor->capacity;
            if (payload_bytes > MESSAGE_CENTER_MAX_PAYLOAD_STORAGE_BYTES - payload_storage_used)
            {
                taskEXIT_CRITICAL();
                return NULL;
            }
            memset(&subscribers[i], 0, sizeof(subscribers[i]));
            subscribers[i].topic_index = (uint8_t)topic_index;
            subscribers[i].endpoint_id = endpoint_id;
            subscribers[i].storage_index = (uint8_t)i;
            subscribers[i].payload_offset = (uint16_t)payload_storage_used;
            payload_storage_used += payload_bytes;
            subscribers[i].active = 1u;
            taskEXIT_CRITICAL();
            return &subscribers[i];
        }
    taskEXIT_CRITICAL();
    return NULL;
}

MessageStatus_t MessagePublisherGetDescriptor(const MessagePublisher_t *publisher, MessageTopicDescriptor_t *output)
{
    if (publisher == NULL || output == NULL || publisher->storage_index >= MESSAGE_CENTER_MAX_SUBSCRIBERS)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready) return MESSAGE_ERR_NOT_INITIALIZED;
    taskENTER_CRITICAL();
    const MessagePublisherStorage_t *handle=&publisher_storage[publisher->storage_index];
    const int index=handle->active ? FindTopicIndex(handle->topic_id) : -1;
    if (index < 0) { taskEXIT_CRITICAL(); return MESSAGE_ERR_TOPIC; }
    *output=topics[index].descriptor;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessageSubscriberGetDescriptor(const MessageSubscriber_t *subscriber, MessageTopicDescriptor_t *output)
{
    if (subscriber == NULL || output == NULL) return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready) return MESSAGE_ERR_NOT_INITIALIZED;
    taskENTER_CRITICAL();
    if (!subscriber->active || subscriber->topic_index >= topic_count) {
        taskEXIT_CRITICAL(); return MESSAGE_ERR_TOPIC;
    }
    *output=topics[subscriber->topic_index].descriptor;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessagePublisherInvalidate(MessagePublisher_t *publisher)
{
    if (publisher==NULL || publisher->storage_index>=MESSAGE_CENTER_MAX_SUBSCRIBERS) return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready) return MESSAGE_ERR_NOT_INITIALIZED;
    taskENTER_CRITICAL();
    const MessagePublisherStorage_t *handle=&publisher_storage[publisher->storage_index];
    const int index=handle->active ? FindTopicIndex(handle->topic_id) : -1;
    if (index<0) { taskEXIT_CRITICAL(); return MESSAGE_ERR_TOPIC; }
    const uint32_t mask=topics[index].descriptor.publisher_mask;
    if (mask==0u || (mask & (mask-1u))!=0u) { taskEXIT_CRITICAL(); return MESSAGE_ERR_RELATION; }
    // 只撤销该单一来源的订阅数据；其他话题和发布顺序不变。
    for (size_t i=0u;i<MESSAGE_CENTER_MAX_SUBSCRIBERS;++i)
        if (subscribers[i].active && subscribers[i].topic_index==(uint8_t)index) {
            subscribers[i].head=0u;
            subscribers[i].count=0u;
        }
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessagePublishAt(MessagePublisher_t *publisher, const void *data,
                                 uint32_t source_time_ms, uint32_t received_time_ms,
                                 uint32_t published_time_ms,
                                 MessageClockDomain_t source_clock)
{
    if (publisher == NULL || data == NULL || publisher->storage_index >= MESSAGE_CENTER_MAX_SUBSCRIBERS)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready)
        return MESSAGE_ERR_NOT_INITIALIZED;

    taskENTER_CRITICAL();
    const MessagePublisherStorage_t *handle = &publisher_storage[publisher->storage_index];
    if (!handle->active)
    {
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_TOPIC;
    }
    const int topic_index = FindTopicIndex(handle->topic_id);
    if (topic_index < 0)
    {
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_TOPIC;
    }
    MessageTopicStorage_t *topic = &topics[topic_index];
    if (source_clock != topic->descriptor.source_clock)
    {
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_CLOCK_DOMAIN;
    }

    if (topic->descriptor.delivery == MESSAGE_DELIVERY_EVENT_FIFO &&
        topic->descriptor.overflow == MESSAGE_OVERFLOW_REJECT_NEWEST)
    {
        for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
        {
            const struct MessageSubscriber *subscriber = &subscribers[i];
            if (subscriber->active && subscriber->topic_index == (uint8_t)topic_index &&
                subscriber->count == topic->descriptor.capacity)
            {
                CountOnce(&topic->stats.rejected);
                taskEXIT_CRITICAL();
                return MESSAGE_ERR_CAPACITY;
            }
        }
    }

    topic->sequence++;
    if (topic->sequence == 0u)
        topic->sequence = 1u;
    CountOnce(&topic->stats.published);

    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
    {
        struct MessageSubscriber *subscriber = &subscribers[i];
        if (!subscriber->active || subscriber->topic_index != (uint8_t)topic_index)
            continue;

        const uint8_t capacity = topic->descriptor.capacity;
        if (subscriber->count == capacity)
        {
            if (topic->descriptor.overflow == MESSAGE_OVERFLOW_REJECT_NEWEST)
            {
                taskEXIT_CRITICAL();
                return MESSAGE_ERR_CAPACITY;
            }
            subscriber->head = (uint8_t)((subscriber->head + 1u) % capacity);
            subscriber->count--;
            CountOnce(&topic->stats.overwritten);
        }

        const uint8_t tail = (uint8_t)((subscriber->head + subscriber->count) % capacity);
        MessageSlot_t *slot = GetSlotMetadata(subscriber, tail);
        memcpy(GetSlotData(subscriber, &topic->descriptor, tail), data,
               topic->descriptor.payload_size);
        slot->freshness.valid = 1u;
        slot->freshness.sequence = topic->sequence;
        slot->freshness.source_time_ms = source_time_ms;
        slot->freshness.received_time_ms = received_time_ms;
        slot->freshness.published_time_ms = published_time_ms;
        slot->freshness.source_clock = (uint8_t)source_clock;
        slot->expiry_counted = 0u;
        subscriber->count++;
    }
    uint8_t backlog = 0u;
    // 统计全部订阅者当前占用，便于判断该话题整体积压。
    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
        if (subscribers[i].active && subscribers[i].topic_index == (uint8_t)topic_index)
            backlog = (uint8_t)(backlog + subscribers[i].count);
    if (backlog > topic->stats.peak_backlog)
        topic->stats.peak_backlog = backlog;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessagePublishNow(MessagePublisher_t *publisher, const void *data)
{
    const uint32_t now_ms = HAL_GetTick();
    return MessagePublishAt(publisher, data, now_ms, now_ms, now_ms, MESSAGE_CLOCK_LOCAL);
}

static uint8_t IsMessageFresh(const MessageTopicDescriptor_t *descriptor,
                              const MessageFreshness_t *freshness, uint32_t now_ms)
{
    const uint32_t age_origin = freshness->source_clock == MESSAGE_CLOCK_REMOTE_RAW ?
                                freshness->received_time_ms : freshness->source_time_ms;
    return (uint8_t)(freshness->valid &&
        (uint32_t)(now_ms - age_origin) <= descriptor->max_age_ms);
}

static MessageStatus_t ValidateRead(MessageSubscriber_t *subscriber, void *output,
                                    size_t output_size, MessageDelivery_t expected_delivery)
{
    if (subscriber == NULL || output == NULL || output_size == 0u)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready)
        return MESSAGE_ERR_NOT_INITIALIZED;
    if (!subscriber->active || subscriber->topic_index >= topic_count)
        return MESSAGE_ERR_TOPIC;
    const MessageTopicDescriptor_t *descriptor = &topics[subscriber->topic_index].descriptor;
    if (output_size != descriptor->payload_size)
        return MESSAGE_ERR_TYPE;
    if (descriptor->delivery != expected_delivery)
        return MESSAGE_ERR_TYPE;
    return MESSAGE_OK;
}

static MessageSlot_t *GetSlotMetadata(const struct MessageSubscriber *subscriber,
                                      uint8_t slot_index)
{
    return &message_slots[subscriber->storage_index][slot_index];
}

static uint8_t *GetSlotData(const struct MessageSubscriber *subscriber,
                            const MessageTopicDescriptor_t *descriptor,
                            uint8_t slot_index)
{
    const size_t offset = (size_t)subscriber->payload_offset +
        (size_t)slot_index * descriptor->payload_size;
    return &payload_storage[offset];
}

MessageStatus_t MessageReadSnapshot(MessageSubscriber_t *subscriber, void *output,
                                    size_t output_size, uint32_t now_ms,
                                    MessageFreshness_t *freshness)
{
    MessageStatus_t status = ValidateRead(subscriber, output, output_size,
                                          MESSAGE_DELIVERY_SNAPSHOT);
    if (status != MESSAGE_OK)
        return status;

    taskENTER_CRITICAL();
    if (subscriber->count == 0u)
    {
        memset(output, 0, output_size);
        if (freshness != NULL)
            memset(freshness, 0, sizeof(*freshness));
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_EMPTY;
    }
    const MessageTopicDescriptor_t *descriptor = &topics[subscriber->topic_index].descriptor;
    MessageSlot_t *slot = GetSlotMetadata(subscriber, subscriber->head);
    if (!IsMessageFresh(descriptor, &slot->freshness, now_ms))
    {
        // 快照保留在队列里供诊断；同一条数据只在首次被读到过期时计数。
        if (slot->expiry_counted == 0u)
        {
            CountOnce(&topics[subscriber->topic_index].stats.expired);
            slot->expiry_counted = 1u;
        }
        memset(output, 0, output_size);
        if (freshness != NULL)
            memset(freshness, 0, sizeof(*freshness));
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_EXPIRED;
    }
    memcpy(output, GetSlotData(subscriber, descriptor, subscriber->head), output_size);
    if (freshness != NULL)
        *freshness = slot->freshness;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessageReadEvent(MessageSubscriber_t *subscriber, void *output,
                                 size_t output_size, uint32_t now_ms,
                                 MessageFreshness_t *freshness)
{
    MessageStatus_t status = ValidateRead(subscriber, output, output_size,
                                          MESSAGE_DELIVERY_EVENT_FIFO);
    if (status != MESSAGE_OK)
        return status;

    taskENTER_CRITICAL();
    const MessageTopicDescriptor_t *descriptor = &topics[subscriber->topic_index].descriptor;
    uint8_t expired_any = 0u;
    // 队列头部过期事件必须丢弃，不能越过顺序执行后面的新事件。
    while (subscriber->count > 0u)
    {
        MessageSlot_t *slot = GetSlotMetadata(subscriber, subscriber->head);
        if (IsMessageFresh(descriptor, &slot->freshness, now_ms))
        {
            memcpy(output, GetSlotData(subscriber, descriptor, subscriber->head), output_size);
            if (freshness != NULL)
                *freshness = slot->freshness;
            subscriber->head = (uint8_t)((subscriber->head + 1u) % descriptor->capacity);
            subscriber->count--;
            taskEXIT_CRITICAL();
            return MESSAGE_OK;
        }
        subscriber->head = (uint8_t)((subscriber->head + 1u) % descriptor->capacity);
        subscriber->count--;
        CountOnce(&topics[subscriber->topic_index].stats.expired);
        expired_any = 1u;
    }
    memset(output, 0, output_size);
    if (freshness != NULL)
        memset(freshness, 0, sizeof(*freshness));
    taskEXIT_CRITICAL();
    return expired_any != 0u ? MESSAGE_ERR_EXPIRED : MESSAGE_ERR_EMPTY;
}

MessageStatus_t MessageGetTopicStats(MessageTopicId_t topic_id, MessageTopicStats_t *stats)
{
    if (stats == NULL)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready)
        return MESSAGE_ERR_NOT_INITIALIZED;
    const int topic_index = FindTopicIndex(topic_id);
    if (topic_index < 0)
        return MESSAGE_ERR_TOPIC;
    taskENTER_CRITICAL();
    *stats = topics[topic_index].stats;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}

MessageStatus_t MessageGetDiagnosticTopicCount(uint8_t *count)
{
    if (count == NULL)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready)
        return MESSAGE_ERR_NOT_INITIALIZED;

    uint8_t candidate = 0u;
    taskENTER_CRITICAL();
    // 目录保持原登记顺序，串口请求据此使用稳定的零起始序号。
    for (size_t i = 0u; i < topic_count; ++i)
        if (topics[i].active && topics[i].descriptor.diagnostics_enabled != 0u)
            ++candidate;
    taskEXIT_CRITICAL();
    *count = candidate;
    return MESSAGE_OK;
}

MessageStatus_t MessageGetTopicDiagnostic(uint8_t index, MessageTopicDiagnostic_t *output)
{
    if (output == NULL)
        return MESSAGE_ERR_ARGUMENT;
    if (!directory_ready)
        return MESSAGE_ERR_NOT_INITIALIZED;

    taskENTER_CRITICAL();
    uint8_t diagnostic_index = 0u;
    int selected_topic = -1;
    // 跳过未开放诊断的话题，使查询序号只对应显式启用项。
    for (size_t i = 0u; i < topic_count; ++i)
    {
        if (!topics[i].active || topics[i].descriptor.diagnostics_enabled == 0u)
            continue;
        if (diagnostic_index == index)
        {
            selected_topic = (int)i;
            break;
        }
        ++diagnostic_index;
    }
    if (selected_topic < 0)
    {
        taskEXIT_CRITICAL();
        return MESSAGE_ERR_RANGE;
    }

    MessageTopicStorage_t *topic = &topics[selected_topic];
    MessageTopicDiagnostic_t candidate = {0};
    candidate.descriptor = topic->descriptor;
    candidate.stats = topic->stats;
    candidate.captured_ms = HAL_GetTick();
    for (size_t i = 0u; i < MESSAGE_CENTER_MAX_SUBSCRIBERS; ++i)
    {
        const MessagePublisherStorage_t *publisher = &publisher_storage[i];
        if (publisher->active && publisher->topic_id == topic->descriptor.topic_id)
        {
            candidate.active_publisher_mask |= UINT32_C(1) << (publisher->endpoint_id - 1u);
            ++candidate.publisher_count;
        }

        const struct MessageSubscriber *subscriber = &subscribers[i];
        if (subscriber->active && subscriber->topic_index == (uint8_t)selected_topic)
        {
            candidate.active_subscriber_mask |= UINT32_C(1) << (subscriber->endpoint_id - 1u);
            ++candidate.subscriber_count;
            candidate.queue_depth = (uint8_t)(candidate.queue_depth + subscriber->count);
            candidate.queue_capacity = (uint8_t)(candidate.queue_capacity +
                                                  topic->descriptor.capacity);
        }
    }
    candidate.queue_remaining = (uint8_t)(candidate.queue_capacity - candidate.queue_depth);
    *output = candidate;
    taskEXIT_CRITICAL();
    return MESSAGE_OK;
}
