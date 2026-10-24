/** @file message_freshness_test.c
 * @brief 验证生产话题快照与事件FIFO的时间、覆盖和过期处理。
 * @note 测试使用独立主机进程；HAL时钟固定，检查Release下仍有效。
 */
#include "test_check.h"
#include "message_center.h"
#include <stdint.h>

uint32_t HAL_GetTick(void) { return 0u; }

static const MessageTopicDescriptor_t topics[] = {
    { 1u, sizeof(uint32_t), 50u, 1u, MESSAGE_DELIVERY_SNAPSHOT,
      MESSAGE_OVERFLOW_REPLACE_OLDEST, MESSAGE_CLOCK_LOCAL, 1u << 0, 1u << 1 },
    { 2u, sizeof(uint32_t), 10u, 2u, MESSAGE_DELIVERY_EVENT_FIFO,
      MESSAGE_OVERFLOW_REJECT_NEWEST, MESSAGE_CLOCK_REMOTE_RAW, 1u << 2, 1u << 3 },
    { 3u, sizeof(uint32_t), 10u, 2u, MESSAGE_DELIVERY_EVENT_FIFO,
      MESSAGE_OVERFLOW_REPLACE_OLDEST, MESSAGE_CLOCK_LOCAL, 1u << 4, 1u << 5 },
};

int main(void)
{
    TEST_CHECK(MessageCenterInit(topics, sizeof(topics) / sizeof(topics[0])) == MESSAGE_OK);
    MessagePublisher_t *pub = MessagePublisherRegister(1u, 1u);
    MessageSubscriber_t *sub = MessageSubscriberRegister(1u, 2u);
    TEST_CHECK(pub != 0 && sub != 0);
    uint32_t sent = 0x12345678u, received = 0u;
    MessageFreshness_t meta;

    MessageStatus_t status = MessageReadSnapshot(sub, &received, sizeof(received), 0u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EMPTY);
    TEST_CHECK(received == 0u);
    status = MessagePublishAt(pub, &sent, 0u, 0u, 4u, MESSAGE_CLOCK_LOCAL);
    TEST_CHECK(status == MESSAGE_OK);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 0u, &meta);
    TEST_CHECK(status == MESSAGE_OK);
    TEST_CHECK(received == sent && meta.valid && meta.sequence == 1u &&
               meta.source_time_ms == 0u && meta.received_time_ms == 0u &&
               meta.published_time_ms == 4u && meta.timestamp_ms == 0u);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 50u, &meta);
    TEST_CHECK(status == MESSAGE_OK);
    TEST_CHECK(received == sent && meta.sequence == 1u);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 51u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EXPIRED);
    TEST_CHECK(received == 0u && !meta.valid);

    /* A new publication refreshes age and replaces the last-value snapshot. */
    sent = 0x87654321u;
    status = MessagePublishAt(pub, &sent, 100u, 100u, 101u, MESSAGE_CLOCK_LOCAL);
    TEST_CHECK(status == MESSAGE_OK);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 100u, &meta);
    TEST_CHECK(status == MESSAGE_OK);
    TEST_CHECK(received == sent && meta.sequence == 2u && meta.timestamp_ms == 100u &&
               meta.published_time_ms == 101u);

    /* Unsigned age arithmetic remains correct across tick wrap. */
    status = MessagePublishAt(pub, &sent, UINT32_MAX - 10u, UINT32_MAX - 10u,
                              UINT32_MAX - 9u, MESSAGE_CLOCK_LOCAL);
    TEST_CHECK(status == MESSAGE_OK);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 5u, &meta);
    TEST_CHECK(status == MESSAGE_OK);
    status = MessageReadSnapshot(sub, &received, sizeof(received), 50u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EXPIRED);
    TEST_CHECK(received == 0u);
    status = MessageReadSnapshot(sub, &received, sizeof(uint16_t), 5u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_TYPE);

    MessagePublisher_t *event_pub = MessagePublisherRegister(2u, 3u);
    MessageSubscriber_t *events = MessageSubscriberRegister(2u, 4u);
    TEST_CHECK(event_pub != NULL && events != NULL);
    sent = 0x11111111u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, 7u, 100u, 101u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
    sent = 0x22222222u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, 8u, 105u, 106u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
    received = 0u;
    status = MessageReadEvent(events, &received, sizeof(received), 110u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == 0x11111111u);
    TEST_CHECK(meta.source_time_ms == 7u && meta.received_time_ms == 100u);
    status = MessageReadEvent(events, &received, sizeof(received), 115u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == 0x22222222u);
    status = MessageReadEvent(events, &received, sizeof(received), 116u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EMPTY && received == 0u);

    /* 远端原始时间不与本机比较；转发和查询也不能改写首次接收时间。 */
    sent = 0x33333333u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, UINT32_MAX - 5u, 200u, 205u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
    status = MessageReadEvent(events, &received, sizeof(received), 210u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == sent &&
               meta.source_time_ms == UINT32_MAX - 5u && meta.received_time_ms == 200u &&
               meta.published_time_ms == 205u);

    /* 满队列时拒绝新事件，不覆盖旧项，也不推进已发布序号。 */
    sent = 0x44444444u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, 300u, 300u, 300u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
    sent = 0x55555555u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, 301u, 301u, 301u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
    sent = 0x66666666u;
    TEST_CHECK(MessagePublishAt(event_pub, &sent, 302u, 302u, 302u,
                                MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_ERR_CAPACITY);
    status = MessageReadEvent(events, &received, sizeof(received), 305u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == 0x44444444u && meta.sequence == 4u);
    status = MessageReadEvent(events, &received, sizeof(received), 305u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == 0x55555555u && meta.sequence == 5u);
    MessageTopicStats_t stats;
    TEST_CHECK(MessageGetTopicStats(2u, &stats) == MESSAGE_OK);
    TEST_CHECK(stats.rejected == 1u && stats.expired == 0u && stats.published == 5u);

    /* 旧事件不能因后续新事件越过队头；到期队头先丢弃，再交付新项。 */
    MessagePublisher_t *replace_pub = MessagePublisherRegister(3u, 5u);
    MessageSubscriber_t *replace_sub = MessageSubscriberRegister(3u, 6u);
    TEST_CHECK(replace_pub != NULL && replace_sub != NULL);
    sent = 0x77777777u;
    TEST_CHECK(MessagePublishAt(replace_pub, &sent, 100u, 100u, 100u,
                                MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    sent = 0x88888888u;
    TEST_CHECK(MessagePublishAt(replace_pub, &sent, 109u, 109u, 109u,
                                MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    status = MessageReadEvent(replace_sub, &received, sizeof(received), 111u, &meta);
    TEST_CHECK(status == MESSAGE_OK && received == 0x88888888u && meta.sequence == 2u);
    status = MessageReadEvent(replace_sub, &received, sizeof(received), 111u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EMPTY && received == 0u);
    TEST_CHECK(MessageGetTopicStats(3u, &stats) == MESSAGE_OK && stats.expired == 1u);

    /* 替换最旧策略保留最新两项。 */
    sent = 1u;
    TEST_CHECK(MessagePublishNow(replace_pub, &sent) == MESSAGE_OK);
    sent = 2u;
    TEST_CHECK(MessagePublishNow(replace_pub, &sent) == MESSAGE_OK);
    sent = 3u;
    TEST_CHECK(MessagePublishNow(replace_pub, &sent) == MESSAGE_OK);
    TEST_CHECK(MessageReadEvent(replace_sub, &received, sizeof(received), 0u, &meta) == MESSAGE_OK);
    TEST_CHECK(received == 2u);
    TEST_CHECK(MessageReadEvent(replace_sub, &received, sizeof(received), 0u, &meta) == MESSAGE_OK);
    TEST_CHECK(received == 3u);
    TEST_CHECK(MessageGetTopicStats(3u, &stats) == MESSAGE_OK && stats.overwritten == 1u);

    /* 过期队列头按FIFO顺序全部丢弃后明确返回EXPIRED。 */
    sent = 4u;
    TEST_CHECK(MessagePublishAt(replace_pub, &sent, 1000u, 1000u, 1000u,
                                MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    sent = 5u;
    TEST_CHECK(MessagePublishAt(replace_pub, &sent, 1001u, 1001u, 1001u,
                                MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    received = 0xabcdef01u;
    status = MessageReadEvent(replace_sub, &received, sizeof(received), 1012u, &meta);
    TEST_CHECK(status == MESSAGE_ERR_EXPIRED && received == 0u && meta.valid == 0u);
    TEST_CHECK(MessageReadEvent(replace_sub, &received, sizeof(received), 1012u,
        &meta) == MESSAGE_ERR_EMPTY);
    TEST_CHECK(MessageGetTopicStats(3u, &stats) == MESSAGE_OK && stats.expired == 3u);
    return 0;
}
