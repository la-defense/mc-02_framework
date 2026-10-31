/** @file chassis_topic_codec_test.c
 * @brief 用固定字节检查跨板底盘话题编码及错误时输出保持。
 */
#include "chassis_topic_codec.h"
#include "test_check.h"
#include <math.h>
#include <string.h>

int main(void)
{
    Chassis_Ctrl_Cmd_s command={.vx=1.0f,.vy=-2.0f,.wz=0.5f,.offset_angle=0.0f,
        .chassis_mode=CHASSIS_NO_FOLLOW,.chassis_speed_buff=-1};
    const uint8_t expected[21]={0,0,0x80,0x3f, 0,0,0,0xc0, 0,0,0,0x3f,
        0,0,0,0, 2, 0xff,0xff,0xff,0xff};
    uint8_t wire[21]={0};
    TEST_CHECK(ChassisCommandEncode(&command,wire,sizeof(wire))==CHASSIS_CODEC_OK);
    TEST_CHECK(memcmp(wire,expected,sizeof(wire))==0);
    Chassis_Ctrl_Cmd_s decoded={0};
    TEST_CHECK(ChassisCommandDecode(wire,sizeof(wire),&decoded)==CHASSIS_CODEC_OK);
    TEST_CHECK(decoded.vx==1.0f && decoded.vy==-2.0f && decoded.wz==0.5f &&
        decoded.chassis_mode==CHASSIS_NO_FOLLOW && decoded.chassis_speed_buff==-1);
    uint8_t unaligned_input[sizeof(command)+1u], unaligned_output[sizeof(command)+1u];
    memcpy(unaligned_input+1u,&command,sizeof(command));
    TEST_CHECK(ChassisCommandBridgeEncode(NULL,unaligned_input+1u,sizeof(command),wire,sizeof(wire)));
    TEST_CHECK(ChassisCommandBridgeDecode(NULL,wire,sizeof(wire),unaligned_output+1u,sizeof(command)));
    TEST_CHECK(memcmp(unaligned_output+1u,&command,sizeof(command))==0);
    Chassis_Ctrl_Cmd_s before=decoded;
    wire[16]=4u;
    TEST_CHECK(ChassisCommandDecode(wire,sizeof(wire),&decoded)==CHASSIS_CODEC_VALUE);
    TEST_CHECK(memcmp(&decoded,&before,sizeof(decoded))==0);
    wire[16]=2u; wire[0]=0u; wire[1]=0u; wire[2]=0x80u; wire[3]=0x7fu;
    TEST_CHECK(ChassisCommandDecode(wire,sizeof(wire),&decoded)==CHASSIS_CODEC_VALUE);
    memset(wire,0xa5,sizeof(wire)); command.vx=NAN;
    TEST_CHECK(ChassisCommandEncode(&command,wire,sizeof(wire))==CHASSIS_CODEC_VALUE);
    TEST_CHECK(wire[0]==0xa5u && wire[20]==0xa5u);
    Chassis_Upload_Data_s feedback={.rest_heat=42u,.bullet_speed=(Bullet_Speed_e)15,.enemy_color=COLOR_BLUE};
    uint8_t feedback_bytes[3];
    TEST_CHECK(ChassisFeedbackEncode(&feedback,feedback_bytes,sizeof(feedback_bytes))==CHASSIS_CODEC_OK);
    TEST_CHECK(memcmp(feedback_bytes,(uint8_t[]){42u,15u,1u},3u)==0);
    feedback_bytes[1]=17u;
    Chassis_Upload_Data_s old=feedback;
    TEST_CHECK(ChassisFeedbackDecode(feedback_bytes,sizeof(feedback_bytes),&feedback)==CHASSIS_CODEC_VALUE);
    TEST_CHECK(memcmp(&feedback,&old,sizeof(feedback))==0);
    return 0;
}
