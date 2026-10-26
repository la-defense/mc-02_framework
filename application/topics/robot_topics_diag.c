/**
 * @file robot_topics_diag.c
 * @brief 将消息中心复制统计编码为USART10按需诊断回复。
 * @note 回调在UART接收任务中执行；每条请求只复制一个已启用话题，不消费消息。
 */
#include "robot_topics_diag.h"

#include "bsp_log_diag.h"
#include "message_center.h"

static void WriteU16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8u);
}

static void WriteU32(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8u);
    output[2] = (uint8_t)(value >> 16u);
    output[3] = (uint8_t)(value >> 24u);
}

static uint8_t RobotTopicsBuildDiagnosticReply(const uint8_t *request,
                                                uint8_t request_length,
                                                uint8_t *reply,
                                                uint8_t reply_capacity,
                                                uint8_t *reply_length)
{
    if (request == NULL || request_length != 1u || reply == NULL || reply_length == NULL ||
        reply_capacity < BSP_LOG_DIAG_TOPIC_HEADER_SIZE)
        return 0u;

    const uint8_t requested_index = request[0];
    uint8_t topic_count = 0u;
    MessageStatus_t status = MessageGetDiagnosticTopicCount(&topic_count);
    reply[0] = (uint8_t)status;
    reply[1] = BSP_LOG_DIAG_TOPIC_SCHEMA_VERSION;
    reply[2] = requested_index;
    reply[3] = topic_count;
    *reply_length = BSP_LOG_DIAG_TOPIC_HEADER_SIZE;
    if (status != MESSAGE_OK || requested_index >= topic_count)
    {
        if (status == MESSAGE_OK)
            reply[0] = MESSAGE_ERR_RANGE;
        return 1u;
    }

    if (reply_capacity < BSP_LOG_DIAG_TOPIC_REPLY_SIZE)
        return 0u;

    MessageTopicDiagnostic_t diagnostic;
    status = MessageGetTopicDiagnostic(requested_index, &diagnostic);
    reply[0] = (uint8_t)status;
    if (status != MESSAGE_OK)
        return 1u;

    uint8_t *record = reply + BSP_LOG_DIAG_TOPIC_HEADER_SIZE;
    size_t offset = 0u;
    WriteU16(record + offset, diagnostic.descriptor.topic_id); offset += 2u;
    WriteU16(record + offset, diagnostic.descriptor.payload_size); offset += 2u;
    WriteU32(record + offset, diagnostic.descriptor.max_age_ms); offset += 4u;
    record[offset++] = diagnostic.descriptor.capacity;
    record[offset++] = (uint8_t)diagnostic.descriptor.delivery;
    record[offset++] = (uint8_t)diagnostic.descriptor.overflow;
    record[offset++] = (uint8_t)diagnostic.descriptor.source_clock;
    WriteU32(record + offset, diagnostic.descriptor.publisher_mask); offset += 4u;
    WriteU32(record + offset, diagnostic.descriptor.subscriber_mask); offset += 4u;
    WriteU32(record + offset, diagnostic.active_publisher_mask); offset += 4u;
    WriteU32(record + offset, diagnostic.active_subscriber_mask); offset += 4u;
    WriteU32(record + offset, diagnostic.stats.published); offset += 4u;
    WriteU32(record + offset, diagnostic.stats.overwritten); offset += 4u;
    WriteU32(record + offset, diagnostic.stats.rejected); offset += 4u;
    WriteU32(record + offset, diagnostic.stats.expired); offset += 4u;
    record[offset++] = diagnostic.stats.peak_backlog;
    record[offset++] = diagnostic.publisher_count;
    record[offset++] = diagnostic.subscriber_count;
    record[offset++] = diagnostic.queue_depth;
    record[offset++] = diagnostic.queue_capacity;
    record[offset++] = diagnostic.queue_remaining;
    WriteU32(record + offset, diagnostic.captured_ms); offset += 4u;

    if (offset != BSP_LOG_DIAG_TOPIC_RECORD_SIZE)
        return 0u;
    *reply_length = (uint8_t)(BSP_LOG_DIAG_TOPIC_HEADER_SIZE + offset);
    return 1u;
}

BSPLogStatus_t RobotTopicsDiagnosticInit(void)
{
    return BSPLogRegisterTopicDiagnosticsHandler(RobotTopicsBuildDiagnosticReply);
}
