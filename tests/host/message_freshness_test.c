#include "message_center.h"
#include <assert.h>
#include <stdint.h>

uint32_t HAL_GetTick(void) { return 0u; }

int main(void)
{
    Publisher_t *pub = PubRegister("fresh", sizeof(uint32_t));
    Subscriber_t *sub = SubRegister("fresh", sizeof(uint32_t));
    assert(pub != 0 && sub != 0);
    uint32_t sent = 0x12345678u, received = 0u;
    MessageFreshness_t meta;

    assert(!SubGetSnapshot(sub, &received, sizeof(received), 0u, 50u, &meta));
    assert(received == 0u);
    assert(PubPushMessageAt(pub, &sent, 0u));
    assert(SubGetSnapshot(sub, &received, sizeof(received), 0u, 50u, &meta));
    assert(received == sent && meta.valid && meta.sequence == 1u && meta.timestamp_ms == 0u);
    assert(SubGetSnapshot(sub, &received, sizeof(received), 50u, 50u, &meta));
    assert(received == sent && meta.sequence == 1u);
    assert(!SubGetSnapshot(sub, &received, sizeof(received), 51u, 50u, &meta));
    assert(received == 0u && !meta.valid);

    /* A new publication refreshes age and replaces the last-value snapshot. */
    sent = 0x87654321u;
    assert(PubPushMessageAt(pub, &sent, 100u));
    assert(SubGetSnapshot(sub, &received, sizeof(received), 100u, 50u, &meta));
    assert(received == sent && meta.sequence == 2u && meta.timestamp_ms == 100u);

    /* Unsigned age arithmetic remains correct across tick wrap. */
    assert(PubPushMessageAt(pub, &sent, UINT32_MAX - 10u));
    assert(SubGetSnapshot(sub, &received, sizeof(received), 5u, 50u, &meta));
    assert(!SubGetSnapshot(sub, &received, sizeof(received), 50u, 50u, &meta));
    assert(received == 0u);
    assert(!SubGetSnapshot(sub, &received, sizeof(uint16_t), 5u, 50u, &meta));
    return 0;
}
