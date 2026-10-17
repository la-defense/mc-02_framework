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
