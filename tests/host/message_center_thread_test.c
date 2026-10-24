/** @file message_center_thread_test.c
 * @brief 用真实主机线程验证发布和读取不会得到混合字段。
 * @note 临界区模型由pthread互斥锁实现；Release构建仍执行所有检查。
 */
#include "test_check.h"
#include "message_center.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

typedef struct
{
    uint32_t sequence;
    uint32_t complement;
    uint32_t marker;
} MessageRecord_t;

static pthread_mutex_t critical_mutex = PTHREAD_MUTEX_INITIALIZER;
static MessagePublisher_t *publisher;
static MessageSubscriber_t *subscriber;
static atomic_int producer_done;
static atomic_int failed;

static MessagePublisher_t *event_publisher;
static MessageSubscriber_t *event_subscriber;

static const MessageTopicDescriptor_t descriptors[] = {
    {
        .topic_id = 1u,
        .payload_size = sizeof(MessageRecord_t),
        .max_age_ms = 100u,
        .capacity = 1u,
        .delivery = MESSAGE_DELIVERY_SNAPSHOT,
        .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 2u,
    },
    {
        .topic_id = 2u,
        .payload_size = sizeof(MessageRecord_t),
        .max_age_ms = 100u,
        .capacity = 4u,
        .delivery = MESSAGE_DELIVERY_EVENT_FIFO,
        .overflow = MESSAGE_OVERFLOW_REJECT_NEWEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 2u,
    },
};

uint32_t HAL_GetTick(void)
{
    return 0u;
}

void HostCriticalEnter(void)
{
    (void)pthread_mutex_lock(&critical_mutex);
}

void HostCriticalExit(void)
{
    (void)pthread_mutex_unlock(&critical_mutex);
}

static void *PublishRecords(void *unused)
{
    (void)unused;
    for (uint32_t sequence = 1u; sequence <= 200000u; ++sequence)
    {
        const MessageRecord_t record = {
            .sequence = sequence,
            .complement = ~sequence,
            .marker = sequence ^ 0xa55aa55au,
        };
        if (MessagePublishAt(publisher, &record, 0u, 0u, sequence,
                             MESSAGE_CLOCK_LOCAL) != MESSAGE_OK)
        {
            atomic_store(&failed, 1);
            break;
        }
        MessageStatus_t event_status = MessagePublishAt(event_publisher, &record,
            0u, 0u, sequence, MESSAGE_CLOCK_LOCAL);
        if (event_status != MESSAGE_OK && event_status != MESSAGE_ERR_CAPACITY)
        {
            atomic_store(&failed, 1);
            break;
        }
    }
    atomic_store(&producer_done, 1);
    return NULL;
}

static void *ReadRecords(void *unused)
{
    (void)unused;
    uint32_t last_event_sequence = 0u;
    for (;;)
    {
        MessageRecord_t record = {0};
        const MessageStatus_t status = MessageReadSnapshot(subscriber, &record,
            sizeof(record), 0u, NULL);
        if (status == MESSAGE_OK &&
            (record.complement != ~record.sequence ||
             record.marker != (record.sequence ^ 0xa55aa55au)))
            atomic_store(&failed, 1);

        MessageRecord_t event = {0};
        const MessageStatus_t event_status = MessageReadEvent(event_subscriber,
            &event, sizeof(event), 0u, NULL);
        if (event_status == MESSAGE_OK)
        {
            if (event.sequence <= last_event_sequence ||
                event.complement != ~event.sequence ||
                event.marker != (event.sequence ^ 0xa55aa55au))
                atomic_store(&failed, 1);
            last_event_sequence = event.sequence;
        }
        else if (event_status != MESSAGE_ERR_EMPTY)
        {
            atomic_store(&failed, 1);
        }

        if (atomic_load(&producer_done) && event_status == MESSAGE_ERR_EMPTY)
            break;
    }
    return NULL;
}

int main(void)
{
    TEST_CHECK(MessageCenterInit(descriptors, sizeof(descriptors) / sizeof(descriptors[0])) == MESSAGE_OK);
    publisher = MessagePublisherRegister(1u, 1u);
    subscriber = MessageSubscriberRegister(1u, 2u);
    event_publisher = MessagePublisherRegister(2u, 1u);
    event_subscriber = MessageSubscriberRegister(2u, 2u);
    TEST_CHECK(publisher != NULL && subscriber != NULL &&
               event_publisher != NULL && event_subscriber != NULL);

    pthread_t producer;
    pthread_t consumer;
    TEST_CHECK(pthread_create(&producer, NULL, PublishRecords, NULL) == 0);
    TEST_CHECK(pthread_create(&consumer, NULL, ReadRecords, NULL) == 0);
    TEST_CHECK(pthread_join(producer, NULL) == 0);
    TEST_CHECK(pthread_join(consumer, NULL) == 0);
    TEST_CHECK(atomic_load(&failed) == 0);

    MessageRecord_t last = {0};
    TEST_CHECK(MessageReadSnapshot(subscriber, &last, sizeof(last), 0u, NULL) == MESSAGE_OK);
    TEST_CHECK(last.sequence == 200000u && last.complement == ~last.sequence);

    MessageTopicStats_t event_stats = {0};
    TEST_CHECK(MessageGetTopicStats(2u, &event_stats) == MESSAGE_OK);
    TEST_CHECK(event_stats.published + event_stats.rejected == 200000u);
    TEST_CHECK(event_stats.overwritten == 0u && event_stats.expired == 0u);
    return 0;
}
