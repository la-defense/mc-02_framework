#include "test_check.h"
#include "message_center.h"
#include <stdint.h>

uint32_t HAL_GetTick(void) { return 0u; }

int main(void)
{
    Publisher_t *pub = PubRegister("fresh", sizeof(uint32_t));
    Subscriber_t *sub = SubRegister("fresh", sizeof(uint32_t));
    TEST_CHECK(pub != 0 && sub != 0);
    uint32_t sent = 0x12345678u, received = 0u;
    MessageFreshness_t meta;

    uint8_t snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 0u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 0u);
    TEST_CHECK(received == 0u);
    uint8_t published = PubPushMessageAt(pub, &sent, 0u);
    TEST_CHECK(published == 1u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 0u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(received == sent && meta.valid && meta.sequence == 1u && meta.timestamp_ms == 0u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 50u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(received == sent && meta.sequence == 1u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 51u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 0u);
    TEST_CHECK(received == 0u && !meta.valid);

    /* A new publication refreshes age and replaces the last-value snapshot. */
    sent = 0x87654321u;
    published = PubPushMessageAt(pub, &sent, 100u);
    TEST_CHECK(published == 1u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 100u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(received == sent && meta.sequence == 2u && meta.timestamp_ms == 100u);

    /* Unsigned age arithmetic remains correct across tick wrap. */
    published = PubPushMessageAt(pub, &sent, UINT32_MAX - 10u);
    TEST_CHECK(published == 1u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 5u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 1u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(received), 50u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 0u);
    TEST_CHECK(received == 0u);
    snapshot_valid = SubGetSnapshot(sub, &received, sizeof(uint16_t), 5u, 50u, &meta);
    TEST_CHECK(snapshot_valid == 0u);
    return 0;
}
