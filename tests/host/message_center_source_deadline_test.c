/** @file message_center_source_deadline_test.c
 * @brief 检查话题发布期限和原始来源期限分别生效。
 */
#include "message_center.h"
#include "test_check.h"
#include <stdint.h>

uint32_t HAL_GetTick(void) { return 0u; } // 测试使用显式本机时间。

int main(void)
{
    const MessageTopicDescriptor_t descriptor = {
        .topic_id = 1u, // 独立测试话题。
        .payload_size = sizeof(uint32_t), // 负载只需一个数值。
        .max_age_ms = 50u, // 任务停止发布后最多保留 50 ms。
        .capacity = 1u, // 控制目标只保留最新值。
        .delivery = MESSAGE_DELIVERY_SNAPSHOT,
        .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 2u,
    };
    TEST_CHECK(MessageCenterInit(&descriptor, 1u) == MESSAGE_OK);
    MessagePublisher_t *publisher = MessagePublisherRegister(1u, 1u);
    MessageSubscriber_t *subscriber = MessageSubscriberRegister(1u, 2u);
    TEST_CHECK(publisher != NULL && subscriber != NULL);
    const uint32_t input = 42u;
    uint32_t output = 0u;
    MessageFreshness_t freshness = {0};
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 100u, 100u, 180u, MESSAGE_CLOCK_LOCAL, 150u) == MESSAGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 230u, &freshness) == MESSAGE_OK);
    TEST_CHECK(output == input && freshness.source_time_ms == 100u && freshness.published_time_ms == 180u &&
               freshness.source_max_age_ms == 150u);
    TEST_CHECK(MessageFreshnessIsValid(&freshness, 230u, 50u)); // 复制给下游后仍保留两种期限。
    TEST_CHECK(!MessageFreshnessIsValid(&freshness, 231u, 50u));
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 231u, NULL) == MESSAGE_ERR_EXPIRED); // 发布者停止后过期。
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 100u, 100u, 231u, MESSAGE_CLOCK_LOCAL, 150u) == MESSAGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 250u, NULL) == MESSAGE_OK); // 原始来源恰好到期。
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 251u, NULL) == MESSAGE_ERR_EXPIRED); // 重复发布不能替来源续期。
    MessageTopicStats_t stats = {0};
    TEST_CHECK(MessageGetTopicStats(1u, &stats) == MESSAGE_OK && stats.expired == 2u); // 两次独立失效各只统计一次。
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 251u, 251u, 251u, MESSAGE_CLOCK_LOCAL, 0u) == MESSAGE_ERR_RANGE);
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 251u, 251u, 251u, MESSAGE_CLOCK_LOCAL, UINT32_MAX) == MESSAGE_ERR_RANGE);
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 252u, 252u, 251u, MESSAGE_CLOCK_LOCAL, 10u) == MESSAGE_ERR_RANGE);
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 260u, 260u, 260u, MESSAGE_CLOCK_LOCAL, 10u) == MESSAGE_OK);
    const uint32_t rejected = 99u;
    TEST_CHECK(MessagePublishAtBounded(publisher, &rejected, 261u, 261u, 260u, MESSAGE_CLOCK_LOCAL, 10u) == MESSAGE_ERR_RANGE);
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 260u, NULL) == MESSAGE_OK && output == input); // 拒绝新数据时保留旧队列。
    TEST_CHECK(MessagePublishAtBounded(publisher, &rejected, 260u, 260u, 260u, MESSAGE_CLOCK_REMOTE_RAW, 10u) == MESSAGE_ERR_CLOCK_DOMAIN);
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, UINT32_MAX - 4u, UINT32_MAX - 4u, 2u, MESSAGE_CLOCK_LOCAL, 10u) == MESSAGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 5u, NULL) == MESSAGE_OK); // 原始时间跨回绕仍可比较。
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 6u, NULL) == MESSAGE_ERR_EXPIRED);
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 20u, 20u, 20u, MESSAGE_CLOCK_LOCAL, 5u) == MESSAGE_OK);
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 20u, &freshness) == MESSAGE_OK);
    const uint32_t initial_sequence = freshness.sequence; // 派生结果引用这次发布。
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 25u, 25u, 25u, MESSAGE_CLOCK_LOCAL, 5u) == MESSAGE_OK);
    TEST_CHECK(MessageSnapshotSourceIsCurrent(1u, initial_sequence, 25u)); // 恰好到期时发布，连续区间仍相同。
    TEST_CHECK(MessagePublishAtBounded(publisher, &input, 31u, 31u, 31u, MESSAGE_CLOCK_LOCAL, 5u) == MESSAGE_OK);
    TEST_CHECK(!MessageSnapshotSourceIsCurrent(1u, initial_sequence, 31u)); // 无人读取过期帧，新鲜数据也不能恢复旧区间。
    TEST_CHECK(MessageReadSnapshot(subscriber, &output, sizeof(output), 31u, &freshness) == MESSAGE_OK);
    TEST_CHECK(MessageSnapshotSourceIsCurrent(1u, freshness.sequence, 31u)); // 新区间可正常派生。
    return 0;
}
