#include "test_check.h"
#include "daemon.h"
#include "message_center.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t HAL_GetTick(void) { return 0u; }

static int allocation_countdown = -1;
static size_t live_allocations;

void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);

void *__wrap_malloc(size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    void *pointer = __real_malloc(size);
    if (pointer != NULL)
        ++live_allocations;
    return pointer;
}

void *__wrap_calloc(size_t count, size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    void *pointer = __real_calloc(count, size);
    if (pointer != NULL)
        ++live_allocations;
    return pointer;
}

void __wrap_free(void *pointer)
{
    if (pointer != NULL)
    {
        TEST_CHECK(live_allocations > 0u);
        --live_allocations;
    }
    __real_free(pointer);
}

static void check_invalid_and_exhausted_daemon(void)
{
    DaemonInstance *invalid = DaemonRegister(NULL);
    TEST_CHECK(invalid == NULL);

    Daemon_Init_Config_s config = {
        .transport_timeout_ms = 100u,
        .data_timeout_ms = 100u,
        .startup_grace_ms = 10u,
        .event_max_age_ms = 100u,
        .source_id = 1u,
        .event_capacity = 1u,
        .consumer_id = 1u,
        .event_policy = DAEMON_EVENT_COALESCED_STATE,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
    };
    allocation_countdown = 0;
    invalid = DaemonRegister(&config);
    TEST_CHECK(invalid == NULL);
    allocation_countdown = -1;

    DaemonInstance *first = DaemonRegister(&config);
    TEST_CHECK(first != NULL);
    for (unsigned i = 1; i < DAEMON_MX_CNT; ++i)
    {
        DaemonInstance *next = DaemonRegister(&config);
        TEST_CHECK(next != NULL);
    }
    size_t before = live_allocations;
    invalid = DaemonRegister(&config);
    TEST_CHECK(invalid == NULL);
    TEST_CHECK(live_allocations == before);
    (void)first;
}

static void check_message_topic_contract(void)
{
    const MessageTopicDescriptor_t valid = {
        .topic_id = 17u,
        .payload_size = sizeof(uint32_t),
        .max_age_ms = 50u,
        .capacity = 1u,
        .delivery = MESSAGE_DELIVERY_SNAPSHOT,
        .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 2u,
    };
    MessageTopicDescriptor_t invalid = valid;
    invalid.payload_size = 0u;
    TEST_CHECK(MessageCenterInit(&invalid, 1u) == MESSAGE_ERR_DIRECTORY);
    invalid = valid;
    invalid.capacity = MESSAGE_CENTER_MAX_QUEUE_DEPTH + 1u;
    TEST_CHECK(MessageCenterInit(&invalid, 1u) == MESSAGE_ERR_DIRECTORY);
    invalid = valid;
    invalid.topic_id = 0u;
    TEST_CHECK(MessageCenterInit(&invalid, 1u) == MESSAGE_ERR_DIRECTORY);

    const size_t before = live_allocations;
    TEST_CHECK(MessageCenterInit(&valid, 1u) == MESSAGE_OK);
    TEST_CHECK(MessageCenterInit(&valid, 1u) == MESSAGE_OK);
    TEST_CHECK(live_allocations == before);
    MessagePublisher_t *publisher = MessagePublisherRegister(17u, 1u);
    MessageSubscriber_t *subscriber = MessageSubscriberRegister(17u, 2u);
    TEST_CHECK(publisher != NULL && subscriber != NULL);
    TEST_CHECK(MessagePublisherRegister(17u, 3u) == NULL);
    TEST_CHECK(MessageSubscriberRegister(17u, 3u) == NULL);
    TEST_CHECK(MessagePublisherRegister(17u, 1u) == publisher);
    TEST_CHECK(MessageSubscriberRegister(17u, 2u) == subscriber);

    uint32_t sent = 0x12345678u, received = 0u;
    TEST_CHECK(MessagePublishAt(publisher, &sent, 0u, 0u, 1u,
                                MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    TEST_CHECK(live_allocations == before);
    TEST_CHECK(MessageReadSnapshot(subscriber, &received, sizeof(received), 0u,
                                   NULL) == MESSAGE_OK);
    TEST_CHECK(received == sent && live_allocations == before);
}

int main(void)
{
    check_invalid_and_exhausted_daemon();
    check_message_topic_contract();
    return 0;
}
