/** @file vision_sp_stream_test.c
 * @brief 通过生产CRC和SP流入口核对分块、多帧、原时间及丢块清历史。
 * @note 固定帧来自独立逐位计算；只运行数值和内存检查，不连接硬件。
 */
#include "vision_protocol.h"
#include "test_check.h"
#include <string.h>

static const uint8_t frame[29] = {
    0x53,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x99,0x71};
static unsigned frames;
static uint32_t expected_ms, expected_generation;
static void Delivered(void *context,const Vision_Recv_s *value,uint32_t source_ms,uint32_t generation)
{
    TEST_CHECK(context == &frames && memcmp(value,frame,sizeof(frame)) == 0);
    TEST_CHECK(source_ms == expected_ms && generation == expected_generation);
    ++frames;
}
static bool Feed(VisionSPStream_t *stream,const uint8_t *data,size_t length,uint32_t source_ms,
                 uint32_t sequence,uint32_t generation,uint32_t now_ms)
{
    return VisionProtocolStreamFeed(stream,data,length,source_ms,sequence,generation,now_ms,
                                    Delivered,&frames,NULL);
}
int main(void)
{
    VisionSPStream_t stream;
    expected_generation=1u;
    TEST_CHECK(VisionProtocolCheckCRC16(frame,sizeof(frame)));
    // 覆盖每个分割点，首块时间0不能被第二块时间或派发时刻替换。
    for(size_t split=1u;split<sizeof(frame);++split)
    {
        VisionProtocolStreamReset(&stream); frames=0u; expected_ms=0u;
        TEST_CHECK(Feed(&stream,frame,split,0u,1u,1u,0u));
        TEST_CHECK(Feed(&stream,frame+split,sizeof(frame)-split,20u,2u,1u,50u));
        TEST_CHECK(frames == 1u && stream.used == 0u);
    }
    VisionProtocolStreamReset(&stream); frames=0u; expected_ms=10u;
    for(size_t i=0u;i<sizeof(frame);++i)
        TEST_CHECK(Feed(&stream,frame+i,1u,10u,(uint32_t)i,1u,10u));
    TEST_CHECK(frames == 1u);
    uint8_t combined[90]; combined[0]=0xffu; combined[1]='S'; combined[2]=0u;
    memcpy(combined+3u,frame,sizeof(frame)); memcpy(combined+32u,frame,sizeof(frame));
    memcpy(combined+61u,frame,sizeof(frame));
    VisionProtocolStreamReset(&stream); frames=0u;
    TEST_CHECK(Feed(&stream,combined,sizeof(combined),10u,1u,1u,10u) && frames == 3u);
    combined[10]^=1u;
    VisionProtocolStreamReset(&stream); frames=0u; uint32_t errors=0u;
    TEST_CHECK(VisionProtocolStreamFeed(&stream,combined,sizeof(combined),10u,1u,1u,10u,
                                        Delivered,&frames,&errors));
    TEST_CHECK(frames == 2u && errors == 1u);
    // 缺块、换连接及超过50ms，都不能用中断前后的两段拼出合法帧。
    for(unsigned reason=0u;reason<3u;++reason)
    {
        VisionProtocolStreamReset(&stream); frames=0u;
        TEST_CHECK(Feed(&stream,frame,10u,10u,1u,1u,10u));
        TEST_CHECK(Feed(&stream,frame+10u,19u,reason==2u?61u:10u,
                        reason==0u?3u:2u,reason==1u?2u:1u,reason==2u?61u:10u));
        TEST_CHECK(frames == 0u);
    }
    VisionProtocolStreamReset(&stream); frames=0u; expected_ms=UINT32_MAX-20u;
    TEST_CHECK(Feed(&stream,frame,10u,UINT32_MAX-20u,UINT32_MAX,1u,UINT32_MAX-20u));
    TEST_CHECK(Feed(&stream,frame+10u,19u,0u,0u,1u,29u) && frames == 1u);
    VisionSPStream_t old=stream; errors=91u;
    TEST_CHECK(!VisionProtocolStreamFeed(&stream,NULL,1u,0u,1u,1u,0u,Delivered,&frames,&errors));
    TEST_CHECK(memcmp(&stream,&old,sizeof(old)) == 0 && errors == 91u);
    TEST_CHECK(!Feed(&stream,frame,0u,0u,1u,1u,0u));
    TEST_CHECK(!Feed(&stream,frame,256u,0u,1u,1u,0u));
    return 0;
}
