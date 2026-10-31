/** @file can_topic_bridge.c
 * @brief 转发消息中心快照，并在远端来源时间可信时发布。
 */
#include "can_topic_bridge.h"
#include <limits.h>
#include <string.h>

CanTopicBridgeStatus_t CanTopicBridgeInit(CanTopicBridge_t *bridge, CanTopicTransport_t *transport,
    MessageSubscriber_t *outgoing, MessagePublisher_t *incoming,
    const CanTopicCodec_t *outgoing_codec, const CanTopicCodec_t *incoming_codec,
    CanTopicMapTime_t map_time, void *map_context)
{
    if (bridge==NULL || transport==NULL || transport->can==NULL || (outgoing==NULL && incoming==NULL))
        return CAN_BRIDGE_ARGUMENT;
    CanTopicBridge_t candidate={0};
    candidate.transport=transport; candidate.outgoing=outgoing; candidate.incoming=incoming;
    candidate.map_time=map_time; candidate.map_context=map_context;
    if (outgoing != NULL && (outgoing_codec==NULL || outgoing_codec->convert==NULL ||
        outgoing_codec->wire_size==0u || outgoing_codec->wire_size>CAN_TOPIC_MAX_PAYLOAD ||
        MessageSubscriberGetDescriptor(outgoing,&candidate.outgoing_desc)!=MESSAGE_OK ||
        candidate.outgoing_desc.delivery!=MESSAGE_DELIVERY_SNAPSHOT ||
        candidate.outgoing_desc.source_clock==MESSAGE_CLOCK_REMOTE_RAW ||
        candidate.outgoing_desc.payload_size>CAN_TOPIC_MAX_PAYLOAD)) return CAN_BRIDGE_CONFIG;
    if (incoming != NULL && (incoming_codec==NULL || incoming_codec->convert==NULL ||
        incoming_codec->wire_size==0u || incoming_codec->wire_size>CAN_TOPIC_MAX_PAYLOAD ||
        MessagePublisherGetDescriptor(incoming,&candidate.incoming_desc)!=MESSAGE_OK ||
        candidate.incoming_desc.delivery!=MESSAGE_DELIVERY_SNAPSHOT ||
        candidate.incoming_desc.source_clock!=MESSAGE_CLOCK_LOCAL_MAPPED ||
        (candidate.incoming_desc.publisher_mask & (candidate.incoming_desc.publisher_mask-1u))!=0u ||
        candidate.incoming_desc.payload_size>CAN_TOPIC_MAX_PAYLOAD)) return CAN_BRIDGE_CONFIG;
    if (outgoing != NULL) candidate.outgoing_codec=*outgoing_codec;
    if (incoming != NULL) candidate.incoming_codec=*incoming_codec;
    *bridge=candidate;
    return CAN_BRIDGE_OK;
}

static uint8_t CanTopicPendingFinished(CanTopicBridge_t *bridge)
{
    if (bridge->pending_request_id==0u) return 1u;
    CAN_TxResult_t result;
    if (!CANGetTxRequestState(bridge->pending_request_id,&result) ||
        (result.pending_frames==0u && (result.state==CAN_TX_RESULT_COMPLETE || result.state==CAN_TX_RESULT_FAILED ||
         result.state==CAN_TX_RESULT_EXPIRED || result.state==CAN_TX_RESULT_CANCELLED || result.state==CAN_TX_RESULT_SUPERSEDED))) {
        bridge->pending_request_id=0u;
        return 1u;
    }
    return 0u;
}

static uint8_t CanTopicCancelPending(CanTopicBridge_t *bridge)
{
    if (bridge->pending_request_id!=0u) (void)CANCancelTxRequest(bridge->pending_request_id);
    // 已标记取消的组仍可能有帧在硬件中，必须等它们结束再提交新组。
    return CanTopicPendingFinished(bridge);
}

CanTopicBridgeStatus_t CanTopicBridgeSend(CanTopicBridge_t *bridge, uint32_t now_ms, uint32_t *request_id)
{
    if (bridge==NULL || request_id==NULL || bridge->outgoing==NULL || bridge->transport==NULL)
        return CAN_BRIDGE_ARGUMENT;
    // 已回收结果不会再被硬件引用；仍在硬件中的终态继续保留编号。
    (void)CanTopicPendingFinished(bridge);
    CanTopicMessage_t message={0}; MessageFreshness_t freshness; uint8_t data[CAN_TOPIC_MAX_PAYLOAD];
    MessageStatus_t read=MessageReadSnapshot(bridge->outgoing,data,
        bridge->outgoing_desc.payload_size,now_ms,&freshness);
    if (read==MESSAGE_ERR_EMPTY || read==MESSAGE_ERR_EXPIRED) {
        (void)CanTopicCancelPending(bridge);
        return read==MESSAGE_ERR_EMPTY ? CAN_BRIDGE_EMPTY : CAN_BRIDGE_EXPIRED;
    }
    if (read!=MESSAGE_OK) {
        (void)CanTopicCancelPending(bridge);
        return CAN_BRIDGE_CONFIG;
    }
    if (freshness.sequence==bridge->last_sent_sequence) return CAN_BRIDGE_EMPTY;
    if (bridge->pending_request_id!=0u) {
        // 先取消旧组余帧；已全部交给硬件时等待终态，不堆积新组。
        if (!CanTopicCancelPending(bridge)) return CAN_BRIDGE_SEND;
    }
    const uint32_t age=(uint32_t)(now_ms-freshness.source_time_ms);
    if (age >= bridge->outgoing_desc.max_age_ms) return CAN_BRIDGE_EXPIRED;
    message.topic_id=bridge->outgoing_desc.topic_id;
    message.length=bridge->outgoing_codec.wire_size;
    message.source_ms=freshness.source_time_ms;
    if (!bridge->outgoing_codec.convert(bridge->outgoing_codec.context,data,bridge->outgoing_desc.payload_size,
                                        message.payload,message.length)) return CAN_BRIDGE_WIRE;
    // 队列期限不能超过来源的剩余时间；达到期限即由队列丢弃。
    CAN_TxSubmitStatus_t status=CanTopicTransportSend(bridge->transport,&message,
        bridge->outgoing_desc.max_age_ms-age,request_id);
    if (status!=CAN_TX_SUBMIT_ACCEPTED) return CAN_BRIDGE_SEND;
    bridge->last_sent_sequence=freshness.sequence;
    bridge->pending_request_id=*request_id;
    return CAN_BRIDGE_OK;
}

CanTopicBridgeStatus_t CanTopicBridgePublish(CanTopicBridge_t *bridge, const CanTopicMessage_t *message,
                                             uint32_t now_ms)
{
    if (bridge==NULL || message==NULL || bridge->incoming==NULL) return CAN_BRIDGE_ARGUMENT;
    if (message->topic_id!=bridge->incoming_desc.topic_id || message->length!=bridge->incoming_codec.wire_size)
        return CAN_BRIDGE_CONFIG;
    // 同一时刻及向后跳变不能覆盖较新的数据；复位后由调用者显式重建桥接。
    uint32_t advance=message->source_ms-bridge->last_remote_source_ms;
    if (bridge->has_remote_source && (advance==0u || advance>=0x80000000u)) return CAN_BRIDGE_WIRE;
    if (bridge->map_time==NULL) return CAN_BRIDGE_UNSYNCED;
    uint32_t local_ms=0u, uncertainty_ms=0u;
    if (!bridge->map_time(bridge->map_context,message->source_ms,now_ms,&local_ms,&uncertainty_ms))
        return CAN_BRIDGE_UNSYNCED;
    if (uncertainty_ms > (uint32_t)INT32_MAX) return CAN_BRIDGE_UNSYNCED;
    // 用最早可能的源时刻判断期限，接收或发布过程不能刷新它。
    uint32_t conservative_ms=local_ms-uncertainty_ms;
    if ((uint32_t)(now_ms-conservative_ms)>bridge->incoming_desc.max_age_ms ||
        (uint32_t)(now_ms-message->first_rx_ms)>bridge->incoming_desc.max_age_ms) return CAN_BRIDGE_EXPIRED;
    uint8_t data[CAN_TOPIC_MAX_PAYLOAD];
    if (!bridge->incoming_codec.convert(bridge->incoming_codec.context,message->payload,message->length,
                                        data,bridge->incoming_desc.payload_size)) return CAN_BRIDGE_WIRE;
    if (MessagePublishAt(bridge->incoming,data,conservative_ms,message->first_rx_ms,
                         now_ms,MESSAGE_CLOCK_LOCAL_MAPPED)!=MESSAGE_OK) return CAN_BRIDGE_PUBLISH;
    bridge->last_remote_source_ms=message->source_ms;
    bridge->last_conservative_ms=conservative_ms;
    bridge->has_remote_source=1u;
    return CAN_BRIDGE_OK;
}

CanTopicBridgeStatus_t CanTopicBridgeReceive(CanTopicBridge_t *bridge, uint32_t now_ms, uint32_t timeout_ms)
{
    if (bridge==NULL || bridge->transport==NULL || bridge->incoming==NULL) return CAN_BRIDGE_ARGUMENT;
    CanTopicMessage_t message;
    CanTopicWireStatus_t status=CanTopicTransportReceive(bridge->transport,now_ms,timeout_ms,&message);
    if (status==CAN_TOPIC_INCOMPLETE) return CAN_BRIDGE_INCOMPLETE;
    if (status!=CAN_TOPIC_OK) return CAN_BRIDGE_WIRE;
    return CanTopicBridgePublish(bridge,&message,now_ms);
}

CanTopicBridgeStatus_t CanTopicBridgeResetPeer(CanTopicBridge_t *bridge)
{
    if (bridge == NULL || bridge->transport == NULL || bridge->incoming == NULL)
        return CAN_BRIDGE_ARGUMENT;
    // 必须先让订阅者读不到旧值，再接受重启后的较小来源时刻。
    if (MessagePublisherInvalidate(bridge->incoming) != MESSAGE_OK)
        return CAN_BRIDGE_PUBLISH;
    memset(&bridge->transport->receiver, 0, sizeof(bridge->transport->receiver));
    bridge->last_remote_source_ms = 0u;
    bridge->last_conservative_ms = 0u;
    bridge->has_remote_source = 0u;
    return CAN_BRIDGE_OK;
}

CanTopicBridgeStatus_t CanTopicBridgeCheckSync(CanTopicBridge_t *bridge, uint32_t now_ms)
{
    if (bridge==NULL || bridge->incoming==NULL || bridge->transport==NULL) return CAN_BRIDGE_ARGUMENT;
    uint32_t mapped=0u, uncertainty=0u;
    if (bridge->map_time!=NULL && bridge->map_time(bridge->map_context,bridge->last_remote_source_ms,
        now_ms,&mapped,&uncertainty) && uncertainty <= (uint32_t)INT32_MAX) {
        const uint32_t earliest=mapped-uncertainty;
        const uint32_t moved_earlier=bridge->last_conservative_ms-earliest;
        // 新映射不能让已发布值的真实年龄大于原先估计，过期也要撤销。
        if (!bridge->has_remote_source ||
            ((uint32_t)(now_ms-earliest)<=bridge->incoming_desc.max_age_ms &&
             (moved_earlier==0u || moved_earlier>=0x80000000u))) return CAN_BRIDGE_OK;
    }
    // 失同步时旧值不可继续被读取，未完成分片也不能在恢复后拼到新消息。
    bridge->transport->receiver.active=0u;
    if (MessagePublisherInvalidate(bridge->incoming)!=MESSAGE_OK) return CAN_BRIDGE_PUBLISH;
    return CAN_BRIDGE_UNSYNCED;
}
