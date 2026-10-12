#include "daemon.h"
#include "message_center.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t HAL_GetTick(void) { return 0u; }

static int allocation_countdown = -1;
static size_t live_allocations;

void *__real_malloc(size_t size);
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

void __wrap_free(void *pointer)
{
    if (pointer != NULL)
    {
        assert(live_allocations > 0u);
        --live_allocations;
    }
    __real_free(pointer);
}

static void check_invalid_and_exhausted_daemon(void)
{
    assert(DaemonRegister(NULL) == NULL);

    Daemon_Init_Config_s config = {0};
    allocation_countdown = 0;
    assert(DaemonRegister(&config) == NULL);
    allocation_countdown = -1;

    DaemonInstance *first = DaemonRegister(&config);
    assert(first != NULL);
    for (unsigned i = 1; i < DAEMON_MX_CNT; ++i)
        assert(DaemonRegister(&config) != NULL);
    size_t before = live_allocations;
    assert(DaemonRegister(&config) == NULL);
    assert(live_allocations == before);
    (void)first;
}

static void check_message_topic_contract(void)
{
    assert(PubRegister(NULL, 1u) == NULL);
    assert(PubRegister("", 1u) == NULL);
    assert(PubRegister("zero-size", 0u) == NULL);

    Publisher_t *original = PubRegister("stable", 4u);
    assert(original != NULL);
    assert(PubRegister("stable", 4u) == original);
    size_t before = live_allocations;
    assert(PubRegister("stable", 5u) == NULL);
    assert(live_allocations == before);

    char too_long[MAX_TOPIC_NAME_LEN + 1u];
    memset(too_long, 'x', sizeof(too_long));
    too_long[sizeof(too_long) - 1u] = '\0';
    assert(PubRegister(too_long, 1u) == NULL);

    for (unsigned i = 0; i < MAX_TOPIC_COUNT - 2u; ++i)
    {
        char name[16];
        (void)snprintf(name, sizeof(name), "topic-%u", i);
        assert(PubRegister(name, 1u) != NULL);
    }
    before = live_allocations;
    assert(PubRegister("overflow", 1u) == NULL);
    assert(live_allocations == before);
}

static void check_subscriber_allocation_rollback(void)
{
    Publisher_t *existing = PubRegister("queue", sizeof(uint32_t));
    assert(existing != NULL);
    size_t before = live_allocations;

    allocation_countdown = 1; /* Subscriber allocation succeeds; queue allocation fails. */
    assert(SubRegister("queue", sizeof(uint32_t)) == NULL);
    allocation_countdown = -1;
    assert(live_allocations == before);
    assert(existing->first_subs == NULL);

    Subscriber_t *sub = SubRegister("queue", sizeof(uint32_t));
    assert(sub != NULL && existing->first_subs == sub);
    uint32_t sent = 0x12345678u;
    uint32_t received = 0u;
    assert(PubPushMessage(existing, &sent) == 1u);
    assert(SubGetMessage(sub, &received) == 1u);
    assert(received == sent);
}

int main(void)
{
    check_invalid_and_exhausted_daemon();
    check_subscriber_allocation_rollback();
    check_message_topic_contract();
    return 0;
}
