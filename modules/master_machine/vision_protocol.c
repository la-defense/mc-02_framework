/** @file vision_protocol.c
* @brief 按现有 SP 格式计算校验，并在接收片段中寻找首个可用完整帧；接口见头文件。
*/
#include "vision_protocol.h"

#include <string.h>

// 沿用双方已使用的逐位算法；每字节先合入 CRC，再处理其中八位。
uint16_t VisionProtocolCRC16(const uint8_t *data, size_t length)
{
    if (data == NULL && length != 0u)
        return 0u;

    uint16_t crc = 0xffffu;
    while (length-- != 0u)
    {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8u; ++bit)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1u) ^ 0x8408u) : (uint16_t)(crc >> 1u);
    }
    return crc;
}

bool VisionProtocolCheckCRC16(const uint8_t *data, size_t length)
{
    if (data == NULL || length < sizeof(uint16_t))
        return false;

    const uint16_t received = (uint16_t)data[length - 2u] |
                             ((uint16_t)data[length - 1u] << 8u);
    return VisionProtocolCRC16(data, length - 2u) == received;
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
