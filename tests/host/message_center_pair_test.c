/** @file message_center_pair_test.c
 * @brief 验证发布和订阅端点成对登记时的容量及回退行为。
 */
#include "message_center.h"
#include "test_check.h"
#include <stdint.h>
#include <string.h>

uint32_t HAL_GetTick(void) { return 0u; } // 主机测试不依赖实时时钟。

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个场景使用新的静态目录。
    const MessageTopicDescriptor_t descriptor = {
        .topic_id = 77u, // 测试使用同一话题的两个方向。
        .payload_size = MESSAGE_CENTER_MAX_PAYLOAD_BYTES, // 每条消息占满允许长度。
        .max_age_ms = 100u, // 数据有效期使用毫秒。
        .capacity = MESSAGE_CENTER_MAX_QUEUE_DEPTH, // 每个订阅端点占用四个数据槽。
        .delivery = MESSAGE_DELIVERY_EVENT_FIFO, // 满队列时保留现有事件。
        .overflow = MESSAGE_OVERFLOW_REJECT_NEWEST, // 超出容量时拒绝新事件。
        .source_clock = MESSAGE_CLOCK_LOCAL, // 所有时间与本机时钟同源。
        .publisher_mask = 0x1ffffu, // 允许端点 1～17 发布。
        .subscriber_mask = 0x1ffffu, // 允许端点 1～17 订阅。
        .diagnostics_enabled = 1u, // 通过只读诊断确认实际登记数。
    };
    TEST_CHECK(MessageCenterInit(&descriptor, 1u) == MESSAGE_OK);
    MessageTopicDescriptor_t copied = {0}; // 在申请任何端点前读取固定声明。
    TEST_CHECK(MessageTopicGetDescriptor(77u, &copied) == MESSAGE_OK && copied.max_age_ms == 100u);
    TEST_CHECK(MessageTopicGetDescriptor(99u, &copied) == MESSAGE_ERR_TOPIC && copied.topic_id == 77u); // 失败不修改原输出。
    MessagePublisher_t *pub = NULL; // 初始化输出，用来检查失败不写入。
    MessageSubscriber_t *sub = NULL; // 订阅输出同样由调用者持有。
    if (!strcmp(argv[1], "payload"))
    {
        for (uint8_t i = 1u; i <= 7u; ++i)
            TEST_CHECK(MessageSubscriberRegister(77u, i) != NULL); // 先占用七份负载区。
        TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 8u, &pub, &sub) == MESSAGE_OK); // 最后一份容量可以成对登记。
        MessagePublisher_t *saved_pub = pub; // 保存已有发布句柄。
        MessageSubscriber_t *saved_sub = sub; // 保存已有订阅句柄。
        TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 8u, &pub, &sub) == MESSAGE_OK && pub == saved_pub && sub == saved_sub); // 重复登记复用原句柄。
        uint8_t message[MESSAGE_CENTER_MAX_PAYLOAD_BYTES] = {42u}; // 以实际事件验证两个句柄可收发。
        uint8_t received[MESSAGE_CENTER_MAX_PAYLOAD_BYTES] = {0}; // 接收缓冲与声明长度一致。
        TEST_CHECK(MessagePublishNow(pub, message) == MESSAGE_OK); // 成对登记的发布句柄能够发送。
        TEST_CHECK(MessageReadEvent(sub, received, sizeof(received), 0u, NULL) == MESSAGE_OK && received[0] == 42u); // 订阅句柄读到对应数据。
        TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 18u, &pub, &sub) == MESSAGE_ERR_RELATION); // 未声明端点不能改变目录。
        TEST_CHECK(MessageEndpointPairRegister(99u, 77u, 8u, &pub, &sub) == MESSAGE_ERR_TOPIC); // 未声明话题同样被拒绝。
        TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 9u, &pub, &sub) == MESSAGE_ERR_CAPACITY); // 负载区不足时两个端点均不新增。
        TEST_CHECK(pub == saved_pub && sub == saved_sub); // 错误不会覆盖已有输出。
        MessageTopicDiagnostic_t diagnostic = {0}; // 查询目录实际登记数量。
        TEST_CHECK(MessageGetTopicDiagnostic(0u, &diagnostic) == MESSAGE_OK);
        TEST_CHECK(diagnostic.publisher_count == 1u && diagnostic.subscriber_count == 8u); // 失败没有留下发布端点。
    }
    else
    {
        TEST_CHECK(!strcmp(argv[1], "publishers")); // 明确区分另一个容量场景。
        for (uint8_t i = 1u; i <= 16u; ++i)
            TEST_CHECK(MessagePublisherRegister(77u, i) != NULL); // 用满发布句柄槽。
        TEST_CHECK(MessageEndpointPairRegister(77u, 77u, 17u, &pub, &sub) == MESSAGE_ERR_CAPACITY); // 发布槽不足时订阅端也不新增。
        TEST_CHECK(pub == NULL && sub == NULL); // 失败不写入调用者输出。
        MessageTopicDiagnostic_t diagnostic = {0}; // 验证不会遗留订阅者。
        TEST_CHECK(MessageGetTopicDiagnostic(0u, &diagnostic) == MESSAGE_OK);
        TEST_CHECK(diagnostic.publisher_count == 16u && diagnostic.subscriber_count == 0u);
    }
    return 0;
}
