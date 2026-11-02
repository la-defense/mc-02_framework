/** @file can_topic_link.c
 * @brief 绑定CAN接收回调、候选桥接及周期同步检查。
 */
#include "can_topic_link.h"
#include "platform_sync.h"
#include "../../bsp/board/board_can_stm32.h"
#include <limits.h>
#include <string.h>

// CANProcessRx在普通任务中派发；消息组装及话题复制不在中断执行。
static void CanTopicLinkReceive(CANInstance *instance)
{
    CanTopicLink_t *link=instance == NULL ? NULL : instance->id;
    if (link==NULL || !link->ready || link->can!=instance || link->bridge.incoming==NULL) return;
    // 对时帧与话题帧共用 CAN ID，但各自检查帧类型和内容。
    if (instance->rx_len>0u && (instance->rx_buff[0]&0xf0u)==0xc0u) {
        if (link->time_sync!=NULL) {
            const uint32_t now=PlatformTimeNowMs(); // 使用与桥接服务相同的本机时钟。
            const CanTimeSyncStatus_t status=CanTimeSyncReceive(link->time_sync,instance->rx_buff,
                instance->rx_len,instance->rx_timestamp_ms,now);
            if (status==CAN_TIME_SYNC_PEER_RESET)
                link->last_receive=CanTopicBridgeResetPeer(&link->bridge);
            else if (status==CAN_TIME_SYNC_OK)
                // 新样本可能使时钟映射失效，不能等到下一次周期才撤销旧值。
                link->last_receive=CanTopicBridgeCheckSync(&link->bridge,now);
        }
        return;
    }
    CanTopicFrame_t frame={0}; // 将本次接收数据复制成与芯片无关的帧。
    frame.length=instance->rx_len; // 保留实际长度供协议检查。
    if (frame.length<=sizeof(frame.bytes)) // 无效长度不读取超出接收缓冲的数据。
        memcpy(frame.bytes,instance->rx_buff,frame.length);
    link->last_receive=CanTopicBridgeReceive(&link->bridge,&frame,instance->rx_timestamp_ms,
                                             PlatformTimeNowMs(),link->assembly_timeout_ms);
}

uint8_t CanTopicLinkInit(CanTopicLink_t *link, PlatformCanBus_t bus, const CAN_Init_Config_s *can_config,
    MessageSubscriber_t *outgoing, MessagePublisher_t *incoming,
    const CanTopicCodec_t *outgoing_codec, const CanTopicCodec_t *incoming_codec, CanTopicMapTime_t map_time,
    void *map_context, uint32_t assembly_timeout_ms)
{
    if (link==NULL || link->ready || can_config==NULL || can_config->can_handle==NULL ||
        BoardCanResolve(bus)!=can_config->can_handle ||
        assembly_timeout_ms==0u || assembly_timeout_ms>INT32_MAX) return 0u;
    CanTopicLink_t candidate={0};
    if (!CanTopicTransportInit(&candidate.transport,bus,can_config->tx_id,link) ||
        CanTopicBridgeInit(&candidate.bridge,&candidate.transport,outgoing,incoming,
                           outgoing_codec,incoming_codec,map_time,map_context)!=CAN_BRIDGE_OK)
        return 0u;
    CAN_Init_Config_s config=*can_config;
    config.id=link; config.can_module_callback=CanTopicLinkReceive;
    CANInstance *instance=CANRegister(&config);
    if (instance==NULL) return 0u;
    // 注册后没有其他可能失败的步骤；对象整体发布前禁止接收任务运行。
    candidate.can=instance; // 保存实际注册端点，供接收回调检查。
    candidate.transport.endpoint_identity=instance; // 发布前将标识换成实际端点。
    candidate.assembly_timeout_ms=assembly_timeout_ms;
    candidate.ready=1u;
    *link=candidate;
    link->bridge.transport=&link->transport;
    return 1u;
}

uint8_t CanTopicLinkAttachTimeSync(CanTopicLink_t *link, CanTimeSync_t *sync)
{
    if (link==NULL || !link->ready || link->can==NULL || link->time_sync!=NULL || link->bridge.incoming==NULL ||
        sync==NULL || !sync->initialized || sync->endpoint_identity!=link->can ||
        sync->tx_id!=link->transport.tx_id || sync->bus!=link->transport.bus ||
        BoardCanResolve(sync->bus)!=link->can->can_handle)
        return 0u;
    if (CanTopicBridgeResetPeer(&link->bridge)!=CAN_BRIDGE_OK)
        return 0u;
    link->bridge.map_time=CanTimeSyncMap;
    link->bridge.map_context=sync;
    link->time_sync=sync;
    return 1u;
}

CanTopicBridgeStatus_t CanTopicLinkService(CanTopicLink_t *link, uint32_t now_ms,
                                          uint32_t *request_id, CanTopicBridgeStatus_t *sync_status)
{
    if (link==NULL || !link->ready || request_id==NULL || sync_status==NULL) return CAN_BRIDGE_ARGUMENT;
    if (link->time_sync!=NULL) (void)CanTimeSyncService(link->time_sync,now_ms);
    CanTopicBridgeStatus_t sync=CAN_BRIDGE_OK;
    if (link->bridge.incoming!=NULL) sync=CanTopicBridgeCheckSync(&link->bridge,now_ms);
    *sync_status=sync;
    if (sync!=CAN_BRIDGE_OK && sync!=CAN_BRIDGE_UNSYNCED) return sync;
    if (link->bridge.outgoing!=NULL) {
        CanTopicBridgeStatus_t send=CanTopicBridgeSend(&link->bridge,now_ms,request_id);
        return send;
    }
    return sync;
}
