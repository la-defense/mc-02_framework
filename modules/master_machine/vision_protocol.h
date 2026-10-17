/** @file vision_protocol.h
* @brief SP 固定帧格式与 CRC16 检查，分别对应上位机 VisionToGimbal、GimbalToVision。
* @note 按字节紧密排列，浮点数据沿用双方相同的 32 位格式；CRC 低字节在前。
*/
#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_COMMAND_FRAME_SIZE 29u
#define VISION_STATUS_FRAME_SIZE 43u

#pragma pack(push, 1)
// 上位机到 MC-02：mode 0 不使用视觉，1 瞄准并停止供弹，2 请求连续供弹。
// yaw/pitch 为左右和上下角度，单位 rad；vel 为 rad/s，acc 为 rad/s²；这里只解析，不检查控制许可。
typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} Vision_Recv_s;

// MC-02 到上位机：mode 为状态模式，q 按 w、x、y、z 排列且无单位。
// yaw/pitch 单位 rad，vel 为 rad/s；bullet_speed 为 m/s，bullet_count 为累计发数。
typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} Vision_Send_s;
#pragma pack(pop)

_Static_assert(sizeof(Vision_Recv_s) == VISION_COMMAND_FRAME_SIZE,
               "Vision command frame must stay 29 bytes");
_Static_assert(sizeof(Vision_Send_s) == VISION_STATUS_FRAME_SIZE,
               "Vision status frame must stay 43 bytes");


/** @brief 检查数据最后两字节是否等于此前内容的 CRC16。
* @param[in] data 完整可读数据，NULL 时返回 false。
* @param[in] length 总字节数，包含末尾两个 CRC 字节，小于 2 时返回 false。
* @return true 为校验一致，false 为输入不符、计算失败或校验不一致。
*/
bool VisionProtocolCheckCRC16(const uint8_t *data, size_t length);

/** @brief 逐字节寻找第一份完整且 CRC 正确的 29 字节命令帧。
* @param[in] data 接收数据，NULL 时失败。
* @param[in] length 可读字节数；不足一帧时失败。
* @param[out] frame 成功时复制整帧；NULL 时失败，其他失败保持原内容。
* @param[out] crc_errors 可选计数，先清零；统计找到 SP 头但 CRC 计算失败或结果错误的候选帧，可为 NULL。
* @return true 为找到并复制，false 为未找到；不保存未收齐的尾部，也不检查角度或模式含义。
*/
bool VisionProtocolFindCommandFrame(const uint8_t *data, size_t length,
                                    Vision_Recv_s *frame, uint32_t *crc_errors);

#endif
