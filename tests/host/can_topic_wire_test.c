/** @file can_topic_wire_test.c
 * @brief 用生产分片接口核对长度、顺序、校验和首次接收时间。
 */
#include "can_topic_wire.h"
#include "test_check.h"
#include <string.h>

int main(void)
{
    CanTopicMessage_t input = {.topic_id=0x1234u, .length=64u, .source_ms=0x12345678u};
    for (unsigned i=0; i<64u; ++i) input.payload[i]=(uint8_t)i;
    CanTopicFrame_t frames[CAN_TOPIC_MAX_FRAMES]; memset(frames,0xa5,sizeof(frames));
    CanTopicFrame_t saved[CAN_TOPIC_MAX_FRAMES]; memcpy(saved,frames,sizeof(saved));
    size_t count=99u;
    TEST_CHECK(CanTopicEncode(&input,0x42u,frames,12u,&count)==CAN_TOPIC_ARGUMENT && count==99u);
    TEST_CHECK(memcmp(frames,saved,sizeof(frames))==0);
    TEST_CHECK(CanTopicEncode(&input, 0x42u, frames, CAN_TOPIC_MAX_FRAMES, &count)==CAN_TOPIC_OK);
    TEST_CHECK(count==13u && frames[0].length==8u && frames[0].bytes[0]==0xa0u &&
               frames[0].bytes[1]==0x42u && frames[0].bytes[2]==0x34u &&
               frames[0].bytes[3]==0x12u && frames[0].bytes[4]==64u);
    CanTopicReceiver_t receiver={0}; CanTopicMessage_t output={0};
    for (size_t i=0; i<count; ++i) {
        CanTopicWireStatus_t result=CanTopicReceive(&receiver,&frames[i],(uint32_t)i,(uint32_t)i,50u,&output);
        TEST_CHECK(result==(i+1u==count ? CAN_TOPIC_OK : CAN_TOPIC_INCOMPLETE));
    }
    TEST_CHECK(output.topic_id==input.topic_id && output.source_ms==input.source_ms &&
               output.first_rx_ms==0u && output.length==64u && memcmp(output.payload,input.payload,64u)==0);
    CanTopicMessage_t previous=output;
    TEST_CHECK(CanTopicReceive(&receiver,&frames[0],20u,20u,50u,&output)==CAN_TOPIC_ORDER);
    receiver.has_completed=0u; // 以下场景分别构造新的消息。
    TEST_CHECK(CanTopicReceive(&receiver,&frames[0],100u,100u,50u,&output)==CAN_TOPIC_INCOMPLETE);
    TEST_CHECK(CanTopicReceive(&receiver,&frames[2],101u,101u,50u,&output)==CAN_TOPIC_ORDER);
    TEST_CHECK(memcmp(&output,&previous,sizeof(output))==0);
    TEST_CHECK(CanTopicReceive(&receiver,&frames[0],200u,200u,50u,&output)==CAN_TOPIC_INCOMPLETE);
    TEST_CHECK(CanTopicReceive(&receiver,&frames[1],251u,251u,50u,&output)==CAN_TOPIC_TIMEOUT);
    TEST_CHECK(memcmp(&output,&previous,sizeof(output))==0);
    TEST_CHECK(CanTopicReceive(&receiver,&frames[0],260u,260u,50u,&output)==CAN_TOPIC_INCOMPLETE);
    TEST_CHECK(CanTopicReceive(&receiver,&frames[1],300u,311u,50u,&output)==CAN_TOPIC_TIMEOUT);
    frames[12].bytes[2]^=1u;
    for (size_t i=0; i<count; ++i) {
        CanTopicWireStatus_t result=CanTopicReceive(&receiver,&frames[i],300u,300u,50u,&output);
        TEST_CHECK(result==(i+1u==count ? CAN_TOPIC_CRC : CAN_TOPIC_INCOMPLETE));
    }
    TEST_CHECK(memcmp(&output,&previous,sizeof(output))==0);
    CanTopicMessage_t short_input={.topic_id=0x1234u,.length=1u,.source_ms=0x12345678u,.payload={0xabu}};
    const uint8_t fixed_first[]={0xa0u,0x21u,0x34u,0x12u,0x01u,0x78u,0x56u,0x34u};
    const uint8_t fixed_last[]={0xa1u,0x21u,0x12u,0xabu,0xebu,0x88u};
    TEST_CHECK(CanTopicEncode(&short_input,0x21u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK && count==2u);
    TEST_CHECK(frames[0].length==sizeof(fixed_first) && memcmp(frames[0].bytes,fixed_first,sizeof(fixed_first))==0);
    TEST_CHECK(frames[1].length==sizeof(fixed_last) && memcmp(frames[1].bytes,fixed_last,sizeof(fixed_last))==0);
    return 0;
}
