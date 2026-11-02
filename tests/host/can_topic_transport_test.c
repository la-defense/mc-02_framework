/** @file can_topic_transport_test.c
 * @brief 检查话题分片与CAN队列的复制、拒绝和接收时间。
 */
#include "can_topic_transport.h"
#include "test_check.h"
#include <string.h>

static CanTopicFrame_t captured[CAN_TOPIC_MAX_FRAMES];
static size_t captured_count;
static CAN_TxSubmitStatus_t submit_status=CAN_TX_SUBMIT_ACCEPTED;
CAN_TxSubmitStatus_t PlatformCanSubmitBatch(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority, uint32_t deadline, uint32_t *id)
{
    TEST_CHECK(bus == PLATFORM_CAN_BUS_1 && source != NULL && priority == CAN_TX_PRIORITY_NORMAL && deadline == 20u);
    if (submit_status != CAN_TX_SUBMIT_ACCEPTED) return submit_status;
    captured_count=count;
    for (size_t i=0u;i<count;++i) {
        TEST_CHECK(frames[i].identifier==0x321u && frames[i].identifier_type==CAN_TX_ID_STANDARD);
        captured[i].length=frames[i].length;
        memcpy(captured[i].bytes,frames[i].data,frames[i].length);
    }
    *id=7u;
    return CAN_TX_SUBMIT_ACCEPTED;
}
CAN_TxSubmitStatus_t PlatformCanSubmitBatchUntil(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority, uint32_t expiry, uint32_t *id)
{
    return PlatformCanSubmitBatch(bus,source,frames,count,priority,expiry,id);
}
int main(void)
{
    int endpoint_identity=0;
    CanTopicTransport_t transport={0};
    // 无效总线不能修改原对象；有效配置随后仍可使用。
    TEST_CHECK(!CanTopicTransportInit(&transport,PLATFORM_CAN_INVALID,0x321u,&endpoint_identity));
    TEST_CHECK(transport.endpoint_identity==NULL);
    TEST_CHECK(CanTopicTransportInit(&transport,PLATFORM_CAN_BUS_1,0x321u,&endpoint_identity));
    CanTopicFrame_t invalid_frame={.length=9u};
    CanTopicMessage_t untouched={.topic_id=42u};
    TEST_CHECK(CanTopicTransportReceive(&transport,&invalid_frame,0u,0u,50u,&untouched)==CAN_TOPIC_FORMAT);
    TEST_CHECK(untouched.topic_id==42u);
    CanTopicMessage_t message={.topic_id=7u,.length=64u,.source_ms=25u};
    for (unsigned i=0u;i<64u;++i) message.payload[i]=(uint8_t)i;
    uint32_t id=99u;
    submit_status=CAN_TX_SUBMIT_FULL;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,20u,&id)==CAN_TX_SUBMIT_FULL);
    TEST_CHECK(id==99u && transport.next_sequence==0u);
    submit_status=CAN_TX_SUBMIT_ACCEPTED;
    TEST_CHECK(CanTopicTransportSend(&transport,&message,20u,&id)==CAN_TX_SUBMIT_ACCEPTED);
    TEST_CHECK(id==7u && transport.next_sequence==1u && captured_count==13u);
    memset(message.payload,0xff,sizeof(message.payload));
    CanTopicMessage_t output={0};
    for (size_t i=0u;i<captured_count;++i) {
        const uint32_t received_ms=(uint32_t)(100u+i);
        CanTopicWireStatus_t status=CanTopicTransportReceive(&transport,&captured[i],received_ms,received_ms,50u,&output);
        TEST_CHECK(status==(i+1u==captured_count ? CAN_TOPIC_OK : CAN_TOPIC_INCOMPLETE));
    }
    TEST_CHECK(output.first_rx_ms==100u && output.source_ms==25u && output.length==64u && output.payload[63]==63u);
    return 0;
}
