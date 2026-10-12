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

    Daemon_Init_Config_s config = {0};
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
    Publisher_t *invalid = PubRegister(NULL, 1u);
    TEST_CHECK(invalid == NULL);
    invalid = PubRegister("", 1u);
    TEST_CHECK(invalid == NULL);
    invalid = PubRegister("zero-size", 0u);
    TEST_CHECK(invalid == NULL);

    Publisher_t *original = PubRegister("stable", 4u);
    TEST_CHECK(original != NULL);
    Publisher_t *same = PubRegister("stable", 4u);
    TEST_CHECK(same == original);
    size_t before = live_allocations;
    invalid = PubRegister("stable", 5u);
    TEST_CHECK(invalid == NULL);
    TEST_CHECK(live_allocations == before);

    char too_long[MAX_TOPIC_NAME_LEN + 1u];
    memset(too_long, 'x', sizeof(too_long));
    too_long[sizeof(too_long) - 1u] = '\0';
    invalid = PubRegister(too_long, 1u);
    TEST_CHECK(invalid == NULL);

    for (unsigned i = 0; i < MAX_TOPIC_COUNT - 2u; ++i)
    {
        char name[16];
        (void)snprintf(name, sizeof(name), "topic-%u", i);
        Publisher_t *topic = PubRegister(name, 1u);
        TEST_CHECK(topic != NULL);
    }
    before = live_allocations;
    invalid = PubRegister("overflow", 1u);
    TEST_CHECK(invalid == NULL);
    TEST_CHECK(live_allocations == before);
}

static void check_subscriber_allocation_rollback(void)
{
    Publisher_t *existing = PubRegister("queue", sizeof(uint32_t));
    TEST_CHECK(existing != NULL);
    size_t before = live_allocations;

    allocation_countdown = 1; /* Subscriber allocation succeeds; queue allocation fails. */
    Subscriber_t *failed_subscriber = SubRegister("queue", sizeof(uint32_t));
    TEST_CHECK(failed_subscriber == NULL);
    allocation_countdown = -1;
    TEST_CHECK(live_allocations == before);
    TEST_CHECK(existing->first_subs == NULL);

    Subscriber_t *sub = SubRegister("queue", sizeof(uint32_t));
    TEST_CHECK(sub != NULL && existing->first_subs == sub);
    uint32_t sent = 0x12345678u;
    uint32_t received = 0u;
    uint8_t published = PubPushMessage(existing, &sent);
    TEST_CHECK(published == 1u);
    uint8_t consumed = SubGetMessage(sub, &received);
    TEST_CHECK(consumed == 1u);
    TEST_CHECK(received == sent);
}

int main(void)
{
    check_invalid_and_exhausted_daemon();
    check_subscriber_allocation_rollback();
    check_message_topic_contract();
    return 0;
}
