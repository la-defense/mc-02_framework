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

static const MessageTopicDescriptor_t descriptor = {
    .topic_id = 1u,
    .payload_size = sizeof(MessageRecord_t),
    .max_age_ms = 100u,
    .capacity = 1u,
    .delivery = MESSAGE_DELIVERY_SNAPSHOT,
    .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST,
    .source_clock = MESSAGE_CLOCK_LOCAL,
    .publisher_mask = 1u,
    .subscriber_mask = 2u,
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
    }
    atomic_store(&producer_done, 1);
    return NULL;
}

static void *ReadRecords(void *unused)
{
    (void)unused;
    do
    {
        MessageRecord_t record = {0};
        const MessageStatus_t status = MessageReadSnapshot(subscriber, &record,
            sizeof(record), 0u, NULL);
        if (status == MESSAGE_OK &&
            (record.complement != ~record.sequence ||
             record.marker != (record.sequence ^ 0xa55aa55au)))
            atomic_store(&failed, 1);
    } while (!atomic_load(&producer_done));
    return NULL;
}

int main(void)
{
    TEST_CHECK(MessageCenterInit(&descriptor, 1u) == MESSAGE_OK);
    publisher = MessagePublisherRegister(1u, 1u);
    subscriber = MessageSubscriberRegister(1u, 2u);
    TEST_CHECK(publisher != NULL && subscriber != NULL);

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
    return 0;
}
