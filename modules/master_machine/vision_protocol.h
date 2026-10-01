#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VISION_COMMAND_FRAME_SIZE 29u
#define VISION_STATUS_FRAME_SIZE 43u

#pragma pack(push, 1)
/* Vision -> MC-02, wire-compatible with sp_vision_25 VisionToGimbal. */
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

/* MC-02 -> vision, wire-compatible with sp_vision_25 GimbalToVision. */
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

uint16_t VisionProtocolCRC16(const uint8_t *data, size_t length);
bool VisionProtocolCheckCRC16(const uint8_t *data, size_t length);

/* Search a byte stream for the first complete valid command frame. */
bool VisionProtocolFindCommandFrame(const uint8_t *data, size_t length,
                                    Vision_Recv_s *frame, uint32_t *crc_errors);

#endif
