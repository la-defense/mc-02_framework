#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

typedef struct
{
    uint8_t online;
    uint8_t mode;
    uint32_t last_rx_ms;
    uint32_t now_ms;
    uint32_t max_age_ms;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
} VisionControlInput_s;

/* Mechanical angle limits must be checked by the caller using robot config. */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

#endif
