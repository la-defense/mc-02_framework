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

typedef struct
{
    uint8_t shoot_enabled;
    uint8_t friction_enabled;
    uint8_t burstfire_enabled;
    float shoot_rate;
} VisionControlShootCommand_s;

/* Mechanical angle limits must be checked by the caller using robot config. */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

/* Builds a safe shooter command on invalid, stale, or disconnected vision input. */
uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command);

#endif
