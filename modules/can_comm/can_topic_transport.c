/** @file can_topic_transport.c
 * @brief 通过生产CAN批量队列提交话题分片，接收时保留首帧时间。
 */
#include "can_topic_transport.h"
#include <limits.h>

// 一组必须能放入最长话题的所有分片，否则不能保证完整入队。
_Static_assert(CAN_TX_MAX_BATCH_FRAMES >= CAN_TOPIC_MAX_FRAMES, "CAN batch too small for topic message");

uint8_t CanTopicTransportInit(CanTopicTransport_t *transport, PlatformCanBus_t bus,
                              uint32_t tx_id, const void *endpoint_identity)
{
    if (transport == NULL || endpoint_identity == NULL || tx_id > 0x7ffu ||
        (bus != PLATFORM_CAN_BUS_1 && bus != PLATFORM_CAN_BUS_2 && bus != PLATFORM_CAN_BUS_3)) return 0u;
    CanTopicTransport_t candidate = {0};
    candidate.bus = bus; // 保存调用者声明的逻辑总线。
    candidate.tx_id = tx_id; // 所有分片使用同一个标准 ID。
    candidate.endpoint_identity = endpoint_identity; // 保留长期端点标识，不访问其内容。
    *transport = candidate;
    return 1u;
}

static CAN_TxSubmitStatus_t CanTopicTransportSendInternal(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t deadline_ms, uint32_t *request_id, uint8_t absolute)
{
    if (transport == NULL || transport->endpoint_identity == NULL || request_id == NULL ||
        (!absolute && (deadline_ms == 0u || deadline_ms > INT32_MAX))) return CAN_TX_SUBMIT_INVALID;
    CanTopicFrame_t encoded[CAN_TOPIC_MAX_FRAMES];
    size_t count = 0u;
    if (CanTopicEncode(message, transport->next_sequence, encoded, CAN_TOPIC_MAX_FRAMES, &count) != CAN_TOPIC_OK)
        return CAN_TX_SUBMIT_INVALID;
    CAN_TxFrame_t frames[CAN_TOPIC_MAX_FRAMES];
    for (size_t i=0u; i<count; ++i) {
        frames[i].identifier=transport->tx_id; // 使用初始化时声明的发送 ID。
        frames[i].identifier_type=CAN_TX_ID_STANDARD; // 话题分片使用标准帧。
        frames[i].data=encoded[i].bytes; // 提交期间引用当前分片，平台会复制。
        frames[i].length=encoded[i].length; // 保留编码后的实际长度。
    }
    // 批量提交复制所有分片；拒绝时不消耗序号，避免假定半包已发送。
    CAN_TxSubmitStatus_t status=absolute ?
        PlatformCanSubmitBatchUntil(transport->bus, transport, frames, count,
                                         CAN_TX_PRIORITY_NORMAL, deadline_ms, request_id) :
        PlatformCanSubmitBatch(transport->bus, transport, frames, count,
                                    CAN_TX_PRIORITY_NORMAL, deadline_ms, request_id);
    if (status == CAN_TX_SUBMIT_ACCEPTED) transport->next_sequence++;
    return status;
}

CAN_TxSubmitStatus_t CanTopicTransportSend(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t deadline_ms, uint32_t *request_id)
{
    return CanTopicTransportSendInternal(transport,message,deadline_ms,request_id,0u);
}

CAN_TxSubmitStatus_t CanTopicTransportSendUntil(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t expiry_ms, uint32_t *request_id)
{
    return CanTopicTransportSendInternal(transport,message,expiry_ms,request_id,1u);
}

CanTopicWireStatus_t CanTopicTransportReceive(CanTopicTransport_t *transport,
    const CanTopicFrame_t *frame, uint32_t received_ms, uint32_t now_ms,
    uint32_t timeout_ms, CanTopicMessage_t *output)
{
    if (transport == NULL || transport->endpoint_identity == NULL || frame == NULL || output == NULL)
        return CAN_TOPIC_ARGUMENT;
    return CanTopicReceive(&transport->receiver, frame, received_ms,
                           now_ms, timeout_ms, output);
}
