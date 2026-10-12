#include "message_center.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* The sentinel is not counted against MAX_TOPIC_COUNT. */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL,
};

static uint8_t IsValidTopic(const char *name, uint8_t data_len)
{
    if (name == NULL || name[0] == '\0' || data_len == 0u)
        return 0u;
    for (size_t i = 0u; i < MAX_TOPIC_NAME_LEN; ++i)
    {
        if (name[i] == '\0')
            return 1u;
    }
    return 0u;
}

static Publisher_t *FindTopic(const char *name, Publisher_t **tail)
{
    Publisher_t *node = &message_center;
    while (node->next_topic_node != NULL)
    {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0)
        {
            if (tail != NULL)
                *tail = node;
            return node;
        }
    }
    if (tail != NULL)
        *tail = node;
    return NULL;
}

static uint8_t TopicCount(void)
{
    uint8_t count = 0u;
    for (Publisher_t *node = message_center.next_topic_node; node != NULL; node = node->next_topic_node)
        count++;
    return count;
}

static Publisher_t *CreateTopic(char *name, uint8_t data_len)
{
    Publisher_t *tail = NULL;
    if (TopicCount() >= MAX_TOPIC_COUNT || FindTopic(name, &tail) != NULL || tail == NULL)
        return NULL;
    Publisher_t *topic = (Publisher_t *)malloc(sizeof(*topic));
    if (topic == NULL)
        return NULL;
    memset(topic, 0, sizeof(*topic));
    topic->data_len = data_len;
    memcpy(topic->topic_name, name, strlen(name) + 1u);
    topic->pub_registered_flag = 1u;
    tail->next_topic_node = topic;
    return topic;
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    if (!IsValidTopic(name, data_len))
        return NULL;
    Publisher_t *topic = FindTopic(name, NULL);
    if (topic != NULL)
    {
        if (topic->data_len != data_len)
            return NULL;
        topic->pub_registered_flag = 1u;
        return topic;
    }
    return CreateTopic(name, data_len);
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    if (!IsValidTopic(name, data_len))
        return NULL;

    Publisher_t *tail = NULL;
    Publisher_t *topic = FindTopic(name, &tail);
    uint8_t create_topic = (uint8_t)(topic == NULL);
    if (topic != NULL && topic->data_len != data_len)
        return NULL;
    if (create_topic && (TopicCount() >= MAX_TOPIC_COUNT || tail == NULL))
        return NULL;

    /* Prepare the complete subscriber before publishing it into any list. */
    Subscriber_t *subscriber = (Subscriber_t *)malloc(sizeof(*subscriber));
    if (subscriber == NULL)
        return NULL;
    memset(subscriber, 0, sizeof(*subscriber));
    subscriber->data_len = data_len;
    for (size_t i = 0u; i < QUEUE_SIZE; ++i)
    {
        subscriber->queue[i] = malloc(data_len);
        if (subscriber->queue[i] == NULL)
        {
            while (i > 0u)
                free(subscriber->queue[--i]);
            free(subscriber);
            return NULL;
        }
    }

    if (create_topic)
    {
        topic = (Publisher_t *)malloc(sizeof(*topic));
        if (topic == NULL)
        {
            for (size_t i = 0u; i < QUEUE_SIZE; ++i)
                free(subscriber->queue[i]);
            free(subscriber);
            return NULL;
        }
        memset(topic, 0, sizeof(*topic));
        topic->data_len = data_len;
        memcpy(topic->topic_name, name, strlen(name) + 1u);
        tail->next_topic_node = topic;
    }

    if (topic->first_subs == NULL)
        topic->first_subs = subscriber;
    else
    {
        Subscriber_t *tail_subscriber = topic->first_subs;
        while (tail_subscriber->next_subs_queue != NULL)
            tail_subscriber = tail_subscriber->next_subs_queue;
        tail_subscriber->next_subs_queue = subscriber;
    }
    return subscriber;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub == NULL || data_ptr == NULL || sub->temp_size == 0u)
        return 0u;
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (uint8_t)((sub->front_idx + 1u) % QUEUE_SIZE);
    sub->temp_size--;
    return 1u;
}

uint8_t SubGetSnapshot(Subscriber_t *sub, void *data_ptr, uint8_t data_len,
                       uint32_t now_ms, uint32_t max_age_ms,
                       MessageFreshness_t *freshness)
{
    if (freshness != NULL)
        memset(freshness, 0, sizeof(*freshness));
    if (data_ptr == NULL || data_len == 0u)
        return 0u;
    memset(data_ptr, 0, data_len);
    if (sub == NULL || sub->data_len != data_len)
        return 0u;

    taskENTER_CRITICAL();
    const MessageFreshness_t snapshot = sub->freshness;
    if (MessageFreshnessIsValid(&snapshot, now_ms, max_age_ms))
    {
        memcpy(data_ptr, sub->queue[0], data_len);
        if (freshness != NULL)
            *freshness = snapshot;
        taskEXIT_CRITICAL();
        return 1u;
    }
    taskEXIT_CRITICAL();
    return 0u;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    return PubPushMessageAt(pub, data_ptr, HAL_GetTick());
}

uint8_t PubPushMessageAt(Publisher_t *pub, void *data_ptr, uint32_t timestamp_ms)
{
    if (pub == NULL || data_ptr == NULL)
        return 0u;
    taskENTER_CRITICAL();
    Subscriber_t *iter = pub->first_subs;
    while (iter != NULL)
    {
        if (iter->temp_size == QUEUE_SIZE)
        {
            iter->front_idx = (uint8_t)((iter->front_idx + 1u) % QUEUE_SIZE);
            iter->temp_size--;
        }
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->freshness.valid = 1u;
        iter->freshness.timestamp_ms = timestamp_ms;
        iter->freshness.sequence++;
        iter->back_idx = (uint8_t)((iter->back_idx + 1u) % QUEUE_SIZE);
        iter->temp_size++;
        iter = iter->next_subs_queue;
    }
    taskEXIT_CRITICAL();
    return 1u;
}
