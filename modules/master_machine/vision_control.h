#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

typedef enum
{
    VISION_YAW_UNCONFIGURED = 0,
    VISION_YAW_CONTINUOUS,
    VISION_YAW_BOUNDED
} VisionYawConstraint_e;

/* Limits are degrees in the final gimbal command frame, not wire radians. */
typedef struct
{
    uint8_t configured;
    VisionYawConstraint_e yaw_constraint;
    float pitch_min_deg;
    float pitch_max_deg;
    float pitch_zero_offset_deg;
    float yaw_min_deg;
    float yaw_max_deg;
} VisionControlLimits_s;

#define VISION_CONTROL_MAX_AGE_MS 50u

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
    const VisionControlLimits_s *limits;
} VisionControlInput_s;

typedef struct
{
    float yaw, pitch;
    float yaw_vel, yaw_acc, pitch_vel, pitch_acc;
} VisionControlGimbalCommand_s;

typedef struct
{
    uint8_t shoot_enabled;
    uint8_t friction_enabled;
    /* Numeric loader_mode_e value; kept uint8_t to make this module host-testable. */
    uint8_t load_mode;
    float shoot_rate;
} VisionControlShootCommand_s;

enum
{
    VISION_CONTROL_LOAD_STOP = 0u,
    VISION_CONTROL_LOAD_REVERSE = 1u,
    VISION_CONTROL_LOAD_SINGLE_BULLET = 2u,
    VISION_CONTROL_LOAD_THREE_BULLETS = 3u,
    VISION_CONTROL_LOAD_BURSTFIRE = 4u
};

/* Missing configuration, stale data or out-of-range commands are unusable. */
uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input);

/* Returns a fully zeroed output on rejection. Wire values remain radians. */
uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command);

/* Builds the vision policy, returning a neutral command when vision is unusable. */
uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command);

/* Returns 1 only when vision was selected; 0 means fallback or command == NULL. */
uint8_t VisionControlSelectShootCommand(const VisionControlInput_s *input,
                                        const VisionControlShootCommand_s *manual_command,
                                        VisionControlShootCommand_s *command);

/* Host-testable equivalents of the production RC dial and keyboard mappings. */
void VisionControlBuildRemoteShootCommand(int16_t dial,
                                          VisionControlShootCommand_s *command);
void VisionControlBuildMouseKeyShootCommand(uint8_t load_cycle,
                                            uint8_t friction_cycle,
                                            VisionControlShootCommand_s *command);

#endif
