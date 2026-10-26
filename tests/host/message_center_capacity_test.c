/** @file message_center_capacity_test.c
 * @brief 验证生产消息中心的固定订阅负载区边界。
 * @note 独立进程从空目录开始；每个订阅占用64字节×4槽，共用2048字节静态区。
 */
#include "test_check.h"
#include "message_center.h"

#include <stdint.h>

uint32_t HAL_GetTick(void)
{
    return 0u;
}

int main(void)
{
    static const MessageTopicDescriptor_t descriptor = {
        .topic_id = 77u,
        .payload_size = MESSAGE_CENTER_MAX_PAYLOAD_BYTES,
        .max_age_ms = 100u,
        .capacity = MESSAGE_CENTER_MAX_QUEUE_DEPTH,
        .delivery = MESSAGE_DELIVERY_EVENT_FIFO,
        .overflow = MESSAGE_OVERFLOW_REJECT_NEWEST,
        .source_clock = MESSAGE_CLOCK_LOCAL,
        .publisher_mask = 1u,
        .subscriber_mask = 0x1ffu,
        .diagnostics_enabled = 0u,
    };
    TEST_CHECK(MessageCenterInit(&descriptor, 1u) == MESSAGE_OK);

    MessageSubscriber_t *subscribers[8];
    for (uint8_t index = 0u; index < 8u; ++index)
    {
        subscribers[index] = MessageSubscriberRegister(77u, (MessageEndpointId_t)(index + 1u));
        TEST_CHECK(subscribers[index] != NULL);
    }
    TEST_CHECK(MessageSubscriberRegister(77u, 9u) == NULL);
    TEST_CHECK(MessageSubscriberRegister(77u, 8u) == subscribers[7]);
    return 0;
}
