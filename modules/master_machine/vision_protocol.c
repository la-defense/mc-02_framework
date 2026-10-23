/** @file vision_protocol.c
* @brief 按现有 SP 格式计算校验，并在接收片段中寻找首个可用完整帧；接口见头文件。
*/
#include "vision_protocol.h"

#include <string.h>
#include "crc.h"

bool VisionProtocolCheckCRC16(const uint8_t *data, size_t length)
{
    if (data == NULL || length < sizeof(uint16_t))
        return false;

    const uint16_t received = (uint16_t)data[length - 2u] |
                             ((uint16_t)data[length - 1u] << 8u);
    uint32_t computed;
    return CRCCompute(CRC16_SP_REFEREE, data, length - 2u, &computed) == CRC_OK &&
           computed == received;
}

bool VisionProtocolFindCommandFrame(const uint8_t *data, size_t length,
                                    Vision_Recv_s *frame, uint32_t *crc_errors)
{
    uint32_t errors = 0u;
    if (crc_errors != NULL)
        *crc_errors = 0u;
    if (data == NULL || frame == NULL || length < sizeof(Vision_Recv_s))
        return false;

    // 只访问仍能放下整帧的位置；头或 CRC 不对时向后移动一个字节继续寻找。
    for (size_t offset = 0; offset <= length - sizeof(Vision_Recv_s); ++offset)
    {
        if (data[offset] != 'S' || data[offset + 1u] != 'P')
            continue;

        if (!VisionProtocolCheckCRC16(data + offset, sizeof(Vision_Recv_s)))
        {
            ++errors;
            continue;
        }

        // 找到第一份正确帧就返回，后面的数据留给调用方决定是否再次处理。
        memcpy(frame, data + offset, sizeof(*frame));
        if (crc_errors != NULL)
            *crc_errors = errors;
        return true;
    }

    if (crc_errors != NULL)
        *crc_errors = errors;
    return false;
}

void VisionProtocolStreamReset(VisionSPStream_t *stream)
{
    if (stream != NULL) memset(stream, 0, sizeof(*stream));
}

// 候选最多29字节，向前移动时同时保留每个字节的原时间。
static void StreamShift(VisionSPStream_t *stream, size_t count)
{
    stream->used -= count;
    memmove(stream->bytes, stream->bytes + count, stream->used);
    memmove(stream->time_ms, stream->time_ms + count, stream->used * sizeof(uint32_t));
}

bool VisionProtocolStreamFeed(VisionSPStream_t *stream, const uint8_t *data, size_t length,
                              uint32_t received_ms, uint32_t sequence, uint32_t generation,
                              uint32_t now_ms, VisionSPFrameCallback_t complete,
                              void *context, uint32_t *crc_errors)
{
    if (stream == NULL || data == NULL || length == 0u || length > 255u || complete == NULL ||
        stream->used >= VISION_COMMAND_FRAME_SIZE)
        return false;
    uint32_t errors = 0u;
    if (crc_errors != NULL) *crc_errors = 0u;
    if ((stream->have_sequence && (sequence != stream->sequence + 1u ||
                                  generation != stream->generation)) ||
        (stream->used != 0u && (uint32_t)(now_ms - stream->time_ms[0]) > 50u))
        stream->used = 0u;
    stream->sequence = sequence;
    stream->generation = generation;
    stream->have_sequence = 1u;
    if ((uint32_t)(now_ms - received_ms) > 50u)
    {
        stream->used = 0u;
        return false;
    }
    // 一次最多255字节；只保存固定候选，错误头或CRC使其前移一字节继续查找。
    for (size_t i = 0u; i < length; ++i)
    {
        stream->bytes[stream->used] = data[i];
        stream->time_ms[stream->used++] = received_ms;
        while (stream->used != 0u)
        {
            if (stream->bytes[0] != 'S' || (stream->used >= 2u && stream->bytes[1] != 'P'))
            { StreamShift(stream, 1u); continue; }
            if (stream->used < VISION_COMMAND_FRAME_SIZE) break;
            if (!VisionProtocolCheckCRC16(stream->bytes, VISION_COMMAND_FRAME_SIZE))
            { ++errors; StreamShift(stream, 1u); continue; }
            Vision_Recv_s frame;
            memcpy(&frame, stream->bytes, sizeof(frame));
            uint32_t first_ms = stream->time_ms[0];
            StreamShift(stream, VISION_COMMAND_FRAME_SIZE);
            complete(context, &frame, first_ms, generation);
        }
    }
    if (crc_errors != NULL) *crc_errors = errors;
    return true;
}
