/** @file can_topic_bridge_test.c
 * @brief 用生产消息中心检查桥接的来源时间、队列失败及失同步行为。
 */
#include "can_topic_link.h"
#include "can_time_sync.h"
#include "chassis_topic_codec.h"
#include "test_check.h"
#include "../../bsp/board/board_can_stm32.h"
#include <string.h>

static uint32_t fake_now_ms;
uint32_t HAL_GetTick(void) { return fake_now_ms; }
static CANInstance registered_can;
static uint8_t reject_registration;
FDCAN_HandleTypeDef *BoardCanResolve(PlatformCanBus_t bus)
{
    return bus == PLATFORM_CAN_BUS_1 ? registered_can.can_handle : NULL;
}
static void (*registered_callback)(CANInstance *);
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (reject_registration) return NULL; // 模拟平台注册失败，验证链路保持原值。
    registered_can.can_handle=config->can_handle;
    registered_can.tx_id=config->tx_id;
    registered_can.id=config->id;
    registered_callback=config->can_module_callback;
    return &registered_can;
}
static CAN_TxSubmitStatus_t tx_result=CAN_TX_SUBMIT_ACCEPTED;
static uint32_t captured_deadline;
static uint32_t captured_expiry;
static CAN_TxResultState_t request_state=CAN_TX_RESULT_COMPLETE;
static uint8_t request_pending;
static unsigned cancellations;
static uint8_t captured_sync_frames[2][8];
static size_t captured_sync_count;
static uint32_t time_offset=100u;
static uint8_t conversion_fail;
static unsigned invalidation_notifications;

static void record_invalidation(void *context)
{
    (void)context;
    ++invalidation_notifications;
}
// 测试话题明确为4个原始字节，生产结构体必须使用各自的字段编解码器。
static uint8_t copy_bytes(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (conversion_fail || input_size!=4u || output_size!=4u) return 0u;
    memcpy(output,input,4u); return 1u;
}
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *handle,const void *source,
    const CAN_TxFrame_t *frames,size_t count,CAN_TxPriority_t priority,uint32_t deadline,uint32_t *id)
{
    (void)handle; (void)source; (void)priority;
    TEST_CHECK(frames!=NULL && count>0u);
    captured_sync_count=count;
    for (size_t i=0u;i<count && i<2u;++i) {
        TEST_CHECK(frames[i].length<=8u);
        memset(captured_sync_frames[i],0,8u);
        memcpy(captured_sync_frames[i],frames[i].data,frames[i].length);
    }
    captured_deadline=deadline;
    if (tx_result==CAN_TX_SUBMIT_ACCEPTED) {
        *id=(frames[0].data[0]&0xf0u)==0xc0u ? 9u : 8u;
        if (*id==8u) request_state=CAN_TX_RESULT_QUEUED;
    }
    return tx_result;
}
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *handle,const void *source,
    const CAN_TxFrame_t *frames,size_t count,CAN_TxPriority_t priority,uint32_t expiry,uint32_t *id)
{
    captured_expiry=expiry;
    const uint32_t remaining=expiry-fake_now_ms;
    if (remaining==0u || remaining>INT32_MAX) return CAN_TX_SUBMIT_INVALID;
    return CANSubmitInterruptibleBatch(handle,source,frames,count,priority,remaining,id);
}
uint8_t CANGetTxRequestState(uint32_t id,CAN_TxResult_t *result)
{
    if ((id!=8u && id!=9u) || result==NULL) return 0u;
    if (id==9u) { memset(result,0,sizeof(*result)); result->state=CAN_TX_RESULT_COMPLETE; return 1u; }
    memset(result,0,sizeof(*result)); result->state=request_state; result->pending_frames=request_pending; return 1u;
}
uint8_t CANCancelTxRequest(uint32_t id)
{
    if (id==9u) return 1u;
    if (id!=8u || (request_state!=CAN_TX_RESULT_QUEUED && request_state!=CAN_TX_RESULT_ACTIVE)) return 0u;
    request_state=CAN_TX_RESULT_CANCELLED; ++cancellations; return 1u;
}
static uint8_t map_time(void *context,uint32_t remote,uint32_t now,uint32_t *local,uint32_t *uncertainty)
{
    (void)now;
    if (*(const uint8_t *)context==0u) return 0u;
    *local=remote+time_offset; *uncertainty=2u;
    return 1u;
}
int main(void)
{
    const MessageTopicDescriptor_t descriptors[]={
        {1u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL,1u,2u,0u},
        {2u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL_MAPPED,4u,8u,0u},
        {3u,4u,50u,1u,MESSAGE_DELIVERY_SNAPSHOT,MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL_MAPPED,48u,64u,0u},
        {4u,sizeof(Chassis_Ctrl_Cmd_s),50u,1u,MESSAGE_DELIVERY_SNAPSHOT,MESSAGE_OVERFLOW_REPLACE_OLDEST,MESSAGE_CLOCK_LOCAL_MAPPED,128u,256u,0u}
    };
    TEST_CHECK(MessageCenterInit(descriptors,4u)==MESSAGE_OK);
    MessagePublisher_t *local=MessagePublisherRegister(1u,1u);
    MessageSubscriber_t *out=MessageSubscriberRegister(1u,2u);
    MessagePublisher_t *remote=MessagePublisherRegister(2u,3u);
    MessageSubscriber_t *in=MessageSubscriberRegister(2u,4u);
    TEST_CHECK(local && out && remote && in);
    MessagePublisher_t *shared=MessagePublisherRegister(3u,5u);
    MessageSubscriber_t *shared_read=MessageSubscriberRegister(3u,7u);
    TEST_CHECK(shared && shared_read);
    uint32_t shared_value=55u, shared_output=0u;
    TEST_CHECK(MessagePublishAt(shared,&shared_value,100u,100u,100u,MESSAGE_CLOCK_LOCAL_MAPPED)==MESSAGE_OK);
    TEST_CHECK(MessagePublisherInvalidate(shared)==MESSAGE_ERR_RELATION);
    TEST_CHECK(MessageReadSnapshot(shared_read,&shared_output,sizeof(shared_output),100u,NULL)==MESSAGE_OK && shared_output==55u);
    FDCAN_HandleTypeDef handle={0}; CANInstance can={.can_handle=&handle,.tx_id=0x321u};
    CanTopicTransport_t transport; TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can.tx_id,&can));
    uint8_t synchronized=0u; CanTopicBridge_t bridge;
    const CanTopicCodec_t codec={copy_bytes,NULL,4u};
    TEST_CHECK(CanTopicBridgeInit(&bridge,&transport,out,remote,&codec,&codec,map_time,&synchronized)==CAN_BRIDGE_OK);
    uint32_t value=123u, id=99u;
    TEST_CHECK(MessagePublishAt(local,&value,100u,100u,100u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    fake_now_ms=110u;
    tx_result=CAN_TX_SUBMIT_FULL;
    TEST_CHECK(CanTopicBridgeSend(&bridge,110u,&id)==CAN_BRIDGE_SEND && id==99u && captured_deadline==40u);
    tx_result=CAN_TX_SUBMIT_ACCEPTED;
    // 即使调用开始时数据未过期，入队前被抢占到截止时刻也不能提交。
    fake_now_ms=150u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,110u,&id)==CAN_BRIDGE_SEND && id==99u);
    fake_now_ms=110u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,110u,&id)==CAN_BRIDGE_OK && id==8u);
    TEST_CHECK(captured_expiry==150u);
    TEST_CHECK(CanTopicBridgeSend(&bridge,111u,&id)==CAN_BRIDGE_EMPTY);
    TEST_CHECK(MessagePublishAt(local,&value,112u,112u,112u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    conversion_fail=1u; id=99u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,113u,&id)==CAN_BRIDGE_WIRE && id==99u);
    TEST_CHECK(cancellations==1u);
    conversion_fail=0u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,113u,&id)==CAN_BRIDGE_OK && id==8u);
    request_state=CAN_TX_RESULT_ACTIVE; request_pending=1u;
    TEST_CHECK(MessagePublishAt(local,&value,114u,114u,114u,MESSAGE_CLOCK_LOCAL)==MESSAGE_OK);
    id=99u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,115u,&id)==CAN_BRIDGE_SEND && id==99u);
    TEST_CHECK(request_state==CAN_TX_RESULT_CANCELLED && request_pending==1u);
    TEST_CHECK(CanTopicBridgeSend(&bridge,116u,&id)==CAN_BRIDGE_SEND && id==99u);
    request_pending=0u;
    TEST_CHECK(CanTopicBridgeSend(&bridge,116u,&id)==CAN_BRIDGE_OK && id==8u);
    CanTopicMessage_t message={.topic_id=2u,.length=4u,.source_ms=10u,.first_rx_ms=115u};
    memcpy(message.payload,&value,sizeof(value));
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,120u)==CAN_BRIDGE_UNSYNCED);
    uint32_t output=0u; MessageFreshness_t freshness;
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),120u,&freshness)==MESSAGE_ERR_EMPTY);
    synchronized=1u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,120u)==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),120u,&freshness)==MESSAGE_OK && output==value &&
        freshness.source_time_ms==108u && freshness.received_time_ms==115u && freshness.published_time_ms==120u);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),158u,&freshness)==MESSAGE_OK);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),159u,&freshness)==MESSAGE_ERR_EXPIRED);
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,120u)==CAN_BRIDGE_WIRE);
    message.source_ms=9u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,120u)==CAN_BRIDGE_WIRE);
    message.source_ms=11u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,160u)==CAN_BRIDGE_EXPIRED);
    message.topic_id=3u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,120u)==CAN_BRIDGE_CONFIG);
    TEST_CHECK(CanTopicBridgeInit(&bridge,&transport,out,remote,&codec,&codec,map_time,&synchronized)==CAN_BRIDGE_OK);
    message.topic_id=2u; message.source_ms=UINT32_MAX-1u; message.first_rx_ms=98u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,100u)==CAN_BRIDGE_OK);
    message.source_ms=1u; message.first_rx_ms=102u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,103u)==CAN_BRIDGE_OK);
    TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,can.tx_id,&can));
    TEST_CHECK(CanTopicBridgeInit(&bridge,&transport,out,remote,&codec,&codec,map_time,&synchronized)==CAN_BRIDGE_OK);
    message.source_ms=20u; message.first_rx_ms=0u;
    CanTopicFrame_t frames[CAN_TOPIC_MAX_FRAMES]; size_t count=0u;
    TEST_CHECK(CanTopicEncode(&message,7u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        can.rx_len=frames[i].length; memcpy(can.rx_buff,frames[i].bytes,can.rx_len);
        can.rx_timestamp_ms=121u+(uint32_t)i;
        CanTopicBridgeStatus_t status=CanTopicBridgeReceive(&bridge,&frames[i],can.rx_timestamp_ms,121u+(uint32_t)i,50u);
        TEST_CHECK(status==(i+1u==count ? CAN_BRIDGE_OK : CAN_BRIDGE_INCOMPLETE));
    }
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),123u,&freshness)==MESSAGE_OK &&
               freshness.source_time_ms==118u && freshness.received_time_ms==121u && output==value);
    time_offset=99u;
    TEST_CHECK(CanTopicBridgeCheckSync(&bridge,123u)==CAN_BRIDGE_EXPIRED);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),123u,&freshness)==MESSAGE_ERR_EMPTY);
    time_offset=100u;
    message.source_ms=21u; message.first_rx_ms=123u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,124u)==CAN_BRIDGE_OK);
    synchronized=0u;
    TEST_CHECK(CanTopicBridgeCheckSync(&bridge,124u)==CAN_BRIDGE_UNSYNCED);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),124u,&freshness)==MESSAGE_ERR_EMPTY);
    synchronized=1u;
    TEST_CHECK(CanTopicBridgeCheckSync(&bridge,125u)==CAN_BRIDGE_OK);
    message.source_ms=22u; message.first_rx_ms=125u;
    TEST_CHECK(CanTopicBridgePublish(&bridge,&message,126u)==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),126u,&freshness)==MESSAGE_OK && output==value);
    CanTopicLink_t link={0}; PlatformCanEndpoint_t config={.can_bus=PLATFORM_CAN_BUS_1,.tx_id=0x321u,.rx_id=0x123u};
    registered_can.can_handle=&handle;
    // 未绑定的逻辑总线不能注册回调。
    config.can_bus=PLATFORM_CAN_BUS_2;
    TEST_CHECK(!CanTopicLinkInit(&link,&config,out,remote,&codec,&codec,map_time,&synchronized,50u));
    TEST_CHECK(registered_callback==NULL);
    config.can_bus=PLATFORM_CAN_BUS_1;
    TEST_CHECK(!CanTopicLinkInit(&link,&config,out,remote,&codec,&codec,map_time,&synchronized,0u));
    reject_registration=1u;
    TEST_CHECK(!CanTopicLinkInit(&link,&config,out,remote,&codec,&codec,map_time,&synchronized,50u));
    TEST_CHECK(!link.ready && !PlatformCanRxIsRegistered(&link.port) && registered_callback==NULL);
    reject_registration=0u;
    TEST_CHECK(CanTopicLinkInit(&link,&config,out,remote,&codec,&codec,map_time,&synchronized,50u));
    TEST_CHECK(!CanTopicLinkInit(&link,&config,out,remote,&codec,&codec,map_time,&synchronized,50u));
    message.source_ms=30u; message.first_rx_ms=0u;
    TEST_CHECK(CanTopicEncode(&message,9u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        registered_can.rx_len=frames[i].length;
        memcpy(registered_can.rx_buff,frames[i].bytes,frames[i].length);
        fake_now_ms=131u+(uint32_t)i; registered_can.rx_timestamp_ms=fake_now_ms;
        registered_callback(&registered_can);
    }
    TEST_CHECK(link.last_receive==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),133u,&freshness)==MESSAGE_OK &&
               freshness.source_time_ms==128u && freshness.received_time_ms==131u);
    // 旧数据超期时已有新消息首片；期限撤销旧值但不能丢弃这组新分片。
    CanTopicBridgeStatus_t sync_status=CAN_BRIDGE_OK;
    message.source_ms=81u;
    TEST_CHECK(CanTopicEncode(&message,10u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        registered_can.rx_len=frames[i].length;
        memcpy(registered_can.rx_buff,frames[i].bytes,frames[i].length);
        fake_now_ms=179u+(uint32_t)i; registered_can.rx_timestamp_ms=fake_now_ms;
        registered_callback(&registered_can);
        (void)CanTopicLinkService(&link,fake_now_ms,&id,&sync_status);
        TEST_CHECK(sync_status==(i==0u ? CAN_BRIDGE_EXPIRED : CAN_BRIDGE_OK));
    }
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),fake_now_ms,&freshness)==MESSAGE_OK && output==value);
    TEST_CHECK(CanTopicBridgePublish(&link.bridge,&message,fake_now_ms)==CAN_BRIDGE_WIRE);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),fake_now_ms,&freshness)==MESSAGE_OK && output==value);
    synchronized=0u; id=99u;
    (void)CanTopicLinkService(&link,190u,&id,&sync_status);
    TEST_CHECK(sync_status==CAN_BRIDGE_UNSYNCED);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),190u,&freshness)==MESSAGE_ERR_EMPTY);
    MessagePublisher_t *chassis_pub=MessagePublisherRegister(4u,8u);
    MessageSubscriber_t *chassis_sub=MessageSubscriberRegister(4u,9u);
    TEST_CHECK(chassis_pub && chassis_sub);
    const CanTopicCodec_t command_decoder={ChassisCommandBridgeDecode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
    CanTopicBridge_t chassis_bridge;
    synchronized=1u;
    TEST_CHECK(CanTopicBridgeInit(&chassis_bridge,&transport,NULL,chassis_pub,NULL,&command_decoder,
                                  map_time,&synchronized)==CAN_BRIDGE_OK);
    TEST_CHECK(CanTopicBridgeSetInvalidateCallback(&chassis_bridge,
        record_invalidation,NULL)==CAN_BRIDGE_OK);
    Chassis_Ctrl_Cmd_s command={.vx=1.0f,.vy=-2.0f,.wz=0.5f,.chassis_mode=CHASSIS_NO_FOLLOW};
    CanTopicMessage_t command_message={.topic_id=4u,.length=CHASSIS_COMMAND_WIRE_BYTES,
        .source_ms=30u,.first_rx_ms=131u};
    TEST_CHECK(ChassisCommandEncode(&command,command_message.payload,sizeof(command_message.payload))==CHASSIS_CODEC_OK);
    TEST_CHECK(CanTopicBridgePublish(&chassis_bridge,&command_message,133u)==CAN_BRIDGE_OK);
    Chassis_Ctrl_Cmd_s decoded={0};
    TEST_CHECK(MessageReadSnapshot(chassis_sub,&decoded,sizeof(decoded),133u,NULL)==MESSAGE_OK &&
               decoded.vx==1.0f && decoded.vy==-2.0f && decoded.chassis_mode==CHASSIS_NO_FOLLOW);
    command_message.source_ms=31u; command_message.payload[2]=0x80u; command_message.payload[3]=0x7fu;
    TEST_CHECK(CanTopicBridgePublish(&chassis_bridge,&command_message,134u)==CAN_BRIDGE_WIRE);
    TEST_CHECK(MessageReadSnapshot(chassis_sub,&decoded,sizeof(decoded),134u,NULL)==MESSAGE_OK && decoded.vx==1.0f);
    // 对端重启后先撤销旧值；新对时完成后允许从较小的远端时刻重新开始。
    TEST_CHECK(CanTopicBridgeResetPeer(&chassis_bridge)==CAN_BRIDGE_OK);
    TEST_CHECK(invalidation_notifications==1u);
    TEST_CHECK(MessageReadSnapshot(chassis_sub,&decoded,sizeof(decoded),134u,NULL)==MESSAGE_ERR_EMPTY);
    time_offset=132u;
    command_message.source_ms=1u; command_message.first_rx_ms=133u;
    TEST_CHECK(ChassisCommandEncode(&command,command_message.payload,sizeof(command_message.payload))==CHASSIS_CODEC_OK);
    TEST_CHECK(CanTopicBridgePublish(&chassis_bridge,&command_message,134u)==CAN_BRIDGE_OK);
    TEST_CHECK(MessageReadSnapshot(chassis_sub,&decoded,sizeof(decoded),134u,NULL)==MESSAGE_OK && decoded.vx==1.0f);
    synchronized=0u;
    TEST_CHECK(CanTopicBridgeCheckSync(&chassis_bridge,135u)==CAN_BRIDGE_UNSYNCED);
    TEST_CHECK(invalidation_notifications==2u);
    TEST_CHECK(MessageReadSnapshot(chassis_sub,&decoded,sizeof(decoded),135u,NULL)==MESSAGE_ERR_EMPTY);

    // 同一个生产 CAN 接收入口先处理对时，再接收经过时间换算的话题。
    CanTopicLink_t timed_link={0};
    TEST_CHECK(CanTopicLinkInit(&timed_link,&config,NULL,remote,NULL,&codec,NULL,NULL,50u));
    CanTimeSync_t local_sync={0}, peer_sync={0};
    FDCAN_HandleTypeDef peer_handle={0};
    CANInstance peer_can={.can_handle=&peer_handle,.tx_id=0x123u};
    CanTimeSync_t wrong_sync={0};
    // 同一对象若配置了其他总线或发送 ID，不能接入当前链路。
    TEST_CHECK(CanTimeSyncInit(&wrong_sync,PLATFORM_CAN_BUS_2,timed_link.transport.tx_id,&timed_link.port));
    TEST_CHECK(!CanTopicLinkAttachTimeSync(&timed_link,&wrong_sync));
    TEST_CHECK(CanTimeSyncInit(&wrong_sync,PLATFORM_CAN_BUS_1,0x7ffu,&timed_link.port));
    TEST_CHECK(!CanTopicLinkAttachTimeSync(&timed_link,&wrong_sync));
    TEST_CHECK(CanTimeSyncInit(&local_sync,PLATFORM_CAN_BUS_1,timed_link.transport.tx_id,&timed_link.port));
    TEST_CHECK(CanTimeSyncInit(&peer_sync,PLATFORM_CAN_BUS_1,peer_can.tx_id,&peer_can));
    CanTopicLink_t missing_endpoint={0};
    missing_endpoint.ready=1u;
    missing_endpoint.bridge.incoming=remote;
    TEST_CHECK(!CanTopicLinkAttachTimeSync(&missing_endpoint,&local_sync));
    TEST_CHECK(CanTopicLinkAttachTimeSync(&timed_link,&local_sync));
    for (uint32_t phase=0u;phase<2u;++phase) {
        if (phase==1u) {
            // 对端复位后从新时钟起点开始；本板的计时不回退。
            memset(&peer_sync,0,sizeof(peer_sync));
            TEST_CHECK(CanTimeSyncInit(&peer_sync,PLATFORM_CAN_BUS_1,peer_can.tx_id,&peer_can));
        }
        for (uint32_t i=0u;i<3u;++i) {
            const uint32_t local_send=1000u+phase*800u+i*200u;
            const uint32_t peer_receive=phase==0u ? local_send+1002u : 2u+i*200u;
            request_state=CAN_TX_RESULT_COMPLETE; request_pending=0u;
            fake_now_ms=local_send; id=99u;
            (void)CanTopicLinkService(&timed_link,local_send,&id,&sync_status);
            TEST_CHECK(captured_sync_count==1u);
            uint8_t probe[8]; memcpy(probe,captured_sync_frames[0],8u);
            TEST_CHECK(CanTimeSyncReceive(&peer_sync,probe,8u,peer_receive,peer_receive)==CAN_TIME_SYNC_OK);
            TEST_CHECK(captured_sync_count==2u);
            uint8_t replies[2][8]; memcpy(replies,captured_sync_frames,sizeof(replies));
            for (size_t j=0u;j<2u;++j) {
                memcpy(registered_can.rx_buff,replies[j],8u);
                registered_can.rx_len=8u; registered_can.rx_timestamp_ms=local_send+4u;
                fake_now_ms=local_send+4u; registered_callback(&registered_can);
            }
            if (phase==1u && i==0u)
                TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),local_send+4u,NULL)==MESSAGE_ERR_EMPTY);
        }
        // 恢复三份样本后，生产拼帧和消息中心能交付新的远端值。
        message.topic_id=2u; message.length=4u; message.source_ms=phase==0u ? 2402u : 402u;
        memcpy(message.payload,&value,sizeof(value));
        TEST_CHECK(CanTopicEncode(&message,(uint8_t)(20u+phase),frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
        const uint32_t base=phase==0u ? 1405u : 2205u;
        for (size_t i=0u;i<count;++i) {
            registered_can.rx_len=frames[i].length;
            memcpy(registered_can.rx_buff,frames[i].bytes,frames[i].length);
            registered_can.rx_timestamp_ms=base+(uint32_t)i; fake_now_ms=registered_can.rx_timestamp_ms;
            registered_callback(&registered_can);
        }
        TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),fake_now_ms,&freshness)==MESSAGE_OK && output==value);
        if (phase==0u) {
            // 在时钟跳变前发布一份新值，确认旧值尚未自然过期。
            message.source_ms=2596u;
            TEST_CHECK(CanTopicEncode(&message,22u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
            for (size_t i=0u;i<count;++i) {
                registered_can.rx_len=frames[i].length;
                memcpy(registered_can.rx_buff,frames[i].bytes,frames[i].length);
                registered_can.rx_timestamp_ms=1596u+(uint32_t)i; fake_now_ms=registered_can.rx_timestamp_ms;
                registered_callback(&registered_can);
            }
            TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),1599u,NULL)==MESSAGE_OK);
            request_state=CAN_TX_RESULT_COMPLETE; fake_now_ms=1600u;
            (void)CanTopicLinkService(&timed_link,1600u,&id,&sync_status);
            uint8_t probe[8]; memcpy(probe,captured_sync_frames[0],8u);
            TEST_CHECK(CanTimeSyncReceive(&peer_sync,probe,8u,2702u,2702u)==CAN_TIME_SYNC_OK);
            uint8_t replies[2][8]; memcpy(replies,captured_sync_frames,sizeof(replies));
            for (size_t j=0u;j<2u;++j) {
                memcpy(registered_can.rx_buff,replies[j],8u);
                registered_can.rx_len=8u; registered_can.rx_timestamp_ms=1604u;
                fake_now_ms=1604u; registered_callback(&registered_can);
            }
            TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),1604u,NULL)==MESSAGE_ERR_EMPTY);
        }
    }
    // 对端重启后主动发送探测，本板无需等待自己的下一次探测就撤销旧消息。
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),2210u,NULL)==MESSAGE_OK);
    memset(&peer_sync,0,sizeof(peer_sync));
    TEST_CHECK(CanTimeSyncInit(&peer_sync,PLATFORM_CAN_BUS_1,peer_can.tx_id,&peer_can));
    TEST_CHECK(CanTimeSyncService(&peer_sync,0u)==CAN_TIME_SYNC_OK);
    memcpy(registered_can.rx_buff,captured_sync_frames[0],8u);
    registered_can.rx_len=8u; registered_can.rx_timestamp_ms=2210u;
    fake_now_ms=2210u; registered_callback(&registered_can);
    TEST_CHECK(MessageReadSnapshot(in,&output,sizeof(output),2210u,NULL)==MESSAGE_ERR_EMPTY);
    return 0;
}
