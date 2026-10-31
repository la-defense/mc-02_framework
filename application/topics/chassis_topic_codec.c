/** @file chassis_topic_codec.c
 * @brief 底盘命令与反馈按小端字段编码；失败时不交付部分结果。
 */
#include "chassis_topic_codec.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128,
               "chassis protocol requires IEEE754 binary32");
_Static_assert(sizeof(int)==4, "chassis command requires 32-bit int");

static void WriteU32(uint8_t *out,uint32_t value)
{
    for (unsigned i=0u;i<4u;++i) out[i]=(uint8_t)(value>>(8u*i));
}
static uint32_t ReadU32(const uint8_t *in)
{
    return (uint32_t)in[0] | (uint32_t)in[1]<<8u | (uint32_t)in[2]<<16u | (uint32_t)in[3]<<24u;
}
static void WriteFloat(uint8_t *out,float value)
{
    uint32_t bits; memcpy(&bits,&value,sizeof(bits)); WriteU32(out,bits);
}
static float ReadFloat(const uint8_t *in)
{
    uint32_t bits=ReadU32(in); float value; memcpy(&value,&bits,sizeof(value)); return value;
}
static uint8_t ValidBullet(int value)
{ return (uint8_t)(value==0 || value==10 || value==15 || value==16 || value==18 || value==30); }

ChassisCodecStatus_t ChassisCommandEncode(const Chassis_Ctrl_Cmd_s *input,uint8_t *output,size_t capacity)
{
    if (input==NULL || output==NULL || capacity<CHASSIS_COMMAND_WIRE_BYTES) return CHASSIS_CODEC_ARGUMENT;
    if (!isfinite(input->vx) || !isfinite(input->vy) || !isfinite(input->wz) || !isfinite(input->offset_angle) ||
        input->chassis_mode<CHASSIS_ZERO_FORCE || input->chassis_mode>CHASSIS_FOLLOW_GIMBAL_YAW) return CHASSIS_CODEC_VALUE;
    uint8_t candidate[CHASSIS_COMMAND_WIRE_BYTES];
    WriteFloat(candidate,input->vx); WriteFloat(candidate+4u,input->vy);
    WriteFloat(candidate+8u,input->wz); WriteFloat(candidate+12u,input->offset_angle);
    candidate[16]=(uint8_t)input->chassis_mode;
    WriteU32(candidate+17u,(uint32_t)input->chassis_speed_buff);
    memcpy(output,candidate,sizeof(candidate)); return CHASSIS_CODEC_OK;
}

ChassisCodecStatus_t ChassisCommandDecode(const uint8_t *input,size_t length,Chassis_Ctrl_Cmd_s *output)
{
    if (input==NULL || output==NULL || length!=CHASSIS_COMMAND_WIRE_BYTES) return CHASSIS_CODEC_ARGUMENT;
    Chassis_Ctrl_Cmd_s candidate={0};
    candidate.vx=ReadFloat(input); candidate.vy=ReadFloat(input+4u);
    candidate.wz=ReadFloat(input+8u); candidate.offset_angle=ReadFloat(input+12u);
    if (!isfinite(candidate.vx) || !isfinite(candidate.vy) || !isfinite(candidate.wz) ||
        !isfinite(candidate.offset_angle) || input[16]>CHASSIS_FOLLOW_GIMBAL_YAW) return CHASSIS_CODEC_VALUE;
    candidate.chassis_mode=(chassis_mode_e)input[16];
    // 通过有符号64位算术还原补码，避免超范围的无符号转有符号转换。
    const uint32_t raw=ReadU32(input+17u);
    candidate.chassis_speed_buff=(int)(raw<=INT32_MAX ? (int64_t)raw : (int64_t)raw-INT64_C(4294967296));
    *output=candidate; return CHASSIS_CODEC_OK;
}

ChassisCodecStatus_t ChassisFeedbackEncode(const Chassis_Upload_Data_s *input,uint8_t *output,size_t capacity)
{
    if (input==NULL || output==NULL || capacity<CHASSIS_FEEDBACK_WIRE_BYTES) return CHASSIS_CODEC_ARGUMENT;
    if (!ValidBullet((int)input->bullet_speed) || input->enemy_color<COLOR_NONE || input->enemy_color>COLOR_RED)
        return CHASSIS_CODEC_VALUE;
    const uint8_t candidate[3]={input->rest_heat,(uint8_t)input->bullet_speed,(uint8_t)input->enemy_color};
    memcpy(output,candidate,sizeof(candidate)); return CHASSIS_CODEC_OK;
}

ChassisCodecStatus_t ChassisFeedbackDecode(const uint8_t *input,size_t length,Chassis_Upload_Data_s *output)
{
    if (input==NULL || output==NULL || length!=CHASSIS_FEEDBACK_WIRE_BYTES) return CHASSIS_CODEC_ARGUMENT;
    if (!ValidBullet(input[1]) || input[2]>COLOR_RED) return CHASSIS_CODEC_VALUE;
    Chassis_Upload_Data_s candidate={0}; candidate.rest_heat=input[0];
    candidate.bullet_speed=(Bullet_Speed_e)input[1]; candidate.enemy_color=(Enemy_Color_e)input[2];
    *output=candidate; return CHASSIS_CODEC_OK;
}

uint8_t ChassisCommandBridgeEncode(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input==NULL || output==NULL || input_size!=sizeof(Chassis_Ctrl_Cmd_s) || output_size!=CHASSIS_COMMAND_WIRE_BYTES) return 0u;
    Chassis_Ctrl_Cmd_s candidate; memcpy(&candidate,input,sizeof(candidate));
    return (uint8_t)(ChassisCommandEncode(&candidate,output,output_size)==CHASSIS_CODEC_OK);
}
uint8_t ChassisCommandBridgeDecode(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input==NULL || output==NULL || output_size!=sizeof(Chassis_Ctrl_Cmd_s)) return 0u;
    Chassis_Ctrl_Cmd_s candidate;
    if (ChassisCommandDecode(input,input_size,&candidate)!=CHASSIS_CODEC_OK) return 0u;
    memcpy(output,&candidate,sizeof(candidate)); return 1u;
}
uint8_t ChassisFeedbackBridgeEncode(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input==NULL || output==NULL || input_size!=sizeof(Chassis_Upload_Data_s) || output_size!=CHASSIS_FEEDBACK_WIRE_BYTES) return 0u;
    Chassis_Upload_Data_s candidate; memcpy(&candidate,input,sizeof(candidate));
    return (uint8_t)(ChassisFeedbackEncode(&candidate,output,output_size)==CHASSIS_CODEC_OK);
}
uint8_t ChassisFeedbackBridgeDecode(void *context,const void *input,size_t input_size,void *output,size_t output_size)
{
    (void)context;
    if (input==NULL || output==NULL || output_size!=sizeof(Chassis_Upload_Data_s)) return 0u;
    Chassis_Upload_Data_s candidate;
    if (ChassisFeedbackDecode(input,input_size,&candidate)!=CHASSIS_CODEC_OK) return 0u;
    memcpy(output,&candidate,sizeof(candidate)); return 1u;
}
