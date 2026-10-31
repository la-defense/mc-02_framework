/** @file can_topic_wire.c
 * @brief 对跨板话题编码和拼接；源时钟映射及话题发布由调用者处理。
 */
#include "can_topic_wire.h"
#include "crc.h"
#include <limits.h>
#include <string.h>

#define WIRE_VERSION 0xa0u
#define WIRE_DATA_PER_FRAME 6u
#define WIRE_OVERHEAD 9u

CanTopicWireStatus_t CanTopicEncode(const CanTopicMessage_t *message, uint8_t sequence,
                                     CanTopicFrame_t *frames, size_t capacity, size_t *count)
{
    if (message == NULL || frames == NULL || count == NULL || message->topic_id == 0u ||
        message->length == 0u || message->length > CAN_TOPIC_MAX_PAYLOAD) return CAN_TOPIC_ARGUMENT;
    const size_t total = (size_t)message->length + WIRE_OVERHEAD;
    const size_t needed = (total + WIRE_DATA_PER_FRAME - 1u) / WIRE_DATA_PER_FRAME;
    if (capacity < needed) return CAN_TOPIC_ARGUMENT;
    uint8_t packet[CAN_TOPIC_PACKET_MAX] = {0};
    packet[0]=(uint8_t)message->topic_id; packet[1]=(uint8_t)(message->topic_id >> 8u);
    packet[2]=message->length;
    for (unsigned i=0u; i<4u; ++i) packet[3u+i]=(uint8_t)(message->source_ms >> (8u*i));
    memcpy(packet+7u,message->payload,message->length);
    uint32_t crc;
    if (CRCCompute(CRC16_SP_REFEREE, packet, total-2u, &crc) != CRC_OK) return CAN_TOPIC_CRC;
    packet[total-2u]=(uint8_t)crc; packet[total-1u]=(uint8_t)(crc >> 8u);
    CanTopicFrame_t candidate[CAN_TOPIC_MAX_FRAMES] = {0};
    // 每帧独立带版本、编号和消息序号，以便拒绝错序或混入的片段。
    for (size_t i=0u; i<needed; ++i) {
        size_t remaining=total-i*WIRE_DATA_PER_FRAME;
        size_t length=remaining < WIRE_DATA_PER_FRAME ? remaining : WIRE_DATA_PER_FRAME;
        candidate[i].bytes[0]=(uint8_t)(WIRE_VERSION | i);
        candidate[i].bytes[1]=sequence;
        memcpy(candidate[i].bytes+2u,packet+i*WIRE_DATA_PER_FRAME,length);
        candidate[i].length=(uint8_t)(length+2u);
    }
    memcpy(frames,candidate,needed*sizeof(*frames));
    *count=needed;
    return CAN_TOPIC_OK;
}

CanTopicWireStatus_t CanTopicReceive(CanTopicReceiver_t *receiver, const CanTopicFrame_t *frame,
                                     uint32_t rx_ms, uint32_t now_ms, uint32_t timeout_ms,
                                     CanTopicMessage_t *output)
{
    if (receiver == NULL || frame == NULL || output == NULL || timeout_ms == 0u || timeout_ms > INT32_MAX)
        return CAN_TOPIC_ARGUMENT;
    if (frame->length < 3u || frame->length > 8u || (frame->bytes[0]&0xf0u) != WIRE_VERSION) {
        receiver->active=0u; return CAN_TOPIC_FORMAT;
    }
    if ((uint32_t)(now_ms-rx_ms) > timeout_ms ||
        (receiver->active && ((uint32_t)(rx_ms-receiver->first_rx_ms) > timeout_ms ||
                              (uint32_t)(now_ms-receiver->first_rx_ms) > timeout_ms))) {
        receiver->active=0u; return CAN_TOPIC_TIMEOUT;
    }
    const uint8_t index=frame->bytes[0]&0x0fu;
    if (!receiver->active) {
        if (index != 0u || frame->length != 8u) return CAN_TOPIC_ORDER;
        // 整条消息重复时不重新发布；正常递增允许255回绕到0。
        if (receiver->has_completed && frame->bytes[1] == receiver->last_sequence) return CAN_TOPIC_ORDER;
        receiver->first_rx_ms=rx_ms; receiver->sequence=frame->bytes[1];
        receiver->expected=0u; receiver->used=0u; receiver->active=1u;
    }
    if (index != receiver->expected || frame->bytes[1] != receiver->sequence) {
        receiver->active=0u; return CAN_TOPIC_ORDER;
    }
    const size_t length=(size_t)frame->length-2u;
    if ((size_t)receiver->used+length > CAN_TOPIC_PACKET_MAX) {
        receiver->active=0u; return CAN_TOPIC_FORMAT;
    }
    memcpy(receiver->packet+receiver->used,frame->bytes+2u,length);
    receiver->used=(uint8_t)(receiver->used+length); receiver->expected++;
    const uint8_t payload_length=receiver->packet[2];
    const size_t total=(size_t)payload_length+WIRE_OVERHEAD;
    if (payload_length == 0u || payload_length > CAN_TOPIC_MAX_PAYLOAD || receiver->used > total ||
        (receiver->used < total && frame->length != 8u)) {
        receiver->active=0u; return CAN_TOPIC_FORMAT;
    }
    if (receiver->used < total) return CAN_TOPIC_INCOMPLETE;
    receiver->active=0u;
    uint32_t crc;
    if (CRCCompute(CRC16_SP_REFEREE,receiver->packet,total-2u,&crc) != CRC_OK ||
        receiver->packet[total-2u] != (uint8_t)crc || receiver->packet[total-1u] != (uint8_t)(crc >> 8u))
        return CAN_TOPIC_CRC;
    CanTopicMessage_t candidate={0};
    candidate.topic_id=(uint16_t)(receiver->packet[0] | ((uint16_t)receiver->packet[1]<<8u));
    if (candidate.topic_id == 0u) return CAN_TOPIC_FORMAT;
    candidate.length=payload_length; candidate.first_rx_ms=receiver->first_rx_ms;
    for (unsigned i=0u; i<4u; ++i) candidate.source_ms |= (uint32_t)receiver->packet[3u+i] << (8u*i);
    memcpy(candidate.payload,receiver->packet+7u,payload_length);
    *output=candidate;
    receiver->last_sequence=receiver->sequence; receiver->has_completed=1u;
    return CAN_TOPIC_OK;
}
