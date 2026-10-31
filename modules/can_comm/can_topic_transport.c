/** @file can_topic_transport.c
 * @brief 通过生产CAN批量队列提交话题分片，接收时保留首帧时间。
 */
#include "can_topic_transport.h"
#include <limits.h>
#include <string.h>

// 一组必须能放入最长话题的所有分片，否则不能保证完整入队。
_Static_assert(CAN_TX_MAX_BATCH_FRAMES >= CAN_TOPIC_MAX_FRAMES, "CAN batch too small for topic message");

uint8_t CanTopicTransportInit(CanTopicTransport_t *transport, CANInstance *can)
{
    if (transport == NULL || can == NULL || can->can_handle == NULL || can->tx_id > 0x7ffu) return 0u;
    CanTopicTransport_t candidate = {0};
    candidate.can = can;
    *transport = candidate;
    return 1u;
}

CAN_TxSubmitStatus_t CanTopicTransportSend(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t deadline_ms, uint32_t *request_id)
{
    if (transport == NULL || transport->can == NULL || request_id == NULL ||
        deadline_ms == 0u || deadline_ms > INT32_MAX) return CAN_TX_SUBMIT_INVALID;
    CanTopicFrame_t encoded[CAN_TOPIC_MAX_FRAMES];
    size_t count = 0u;
    if (CanTopicEncode(message, transport->next_sequence, encoded, CAN_TOPIC_MAX_FRAMES, &count) != CAN_TOPIC_OK)
        return CAN_TX_SUBMIT_INVALID;
    CAN_TxFrame_t frames[CAN_TOPIC_MAX_FRAMES];
    for (size_t i=0u; i<count; ++i) {
        frames[i].identifier=transport->can->tx_id;
        frames[i].identifier_type=CAN_TX_ID_STANDARD;
        frames[i].data=encoded[i].bytes;
        frames[i].length=encoded[i].length;
    }
    // 批量提交复制所有分片；拒绝时不消耗序号，避免假定半包已发送。
    CAN_TxSubmitStatus_t status=CANSubmitInterruptibleBatch(transport->can->can_handle, transport,
        frames, count, CAN_TX_PRIORITY_NORMAL, deadline_ms, request_id);
    if (status == CAN_TX_SUBMIT_ACCEPTED) transport->next_sequence++;
    return status;
}

CanTopicWireStatus_t CanTopicTransportReceive(CanTopicTransport_t *transport,
    uint32_t now_ms, uint32_t timeout_ms, CanTopicMessage_t *output)
{
    if (transport == NULL || transport->can == NULL || output == NULL) return CAN_TOPIC_ARGUMENT;
    CanTopicFrame_t frame={0};
    frame.length=transport->can->rx_len;
    if (frame.length <= sizeof(frame.bytes)) memcpy(frame.bytes,transport->can->rx_buff,frame.length);
    return CanTopicReceive(&transport->receiver, &frame, transport->can->rx_timestamp_ms,
                           now_ms, timeout_ms, output);
}
