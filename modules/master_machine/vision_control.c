#include "vision_control.h"

#include <math.h>
#include <stddef.h>

static void ResetShootCommand(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = 0u;
    command->friction_enabled = 0u;
    command->load_mode = VISION_CONTROL_LOAD_STOP;
    command->shoot_rate = 0.0f;
}

static void EnableManualShootIfRequested(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = (uint8_t)(command->friction_enabled ||
                                       command->load_mode != VISION_CONTROL_LOAD_STOP);
}

uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input)
{
    const float radians_to_degrees = 57.29577951308232f;
    if (input == NULL || input->online == 0u ||
        (input->mode != 1u && input->mode != 2u) ||
        (uint32_t)(input->now_ms - input->last_rx_ms) > input->max_age_ms)
        return 0u;

    return (uint8_t)(isfinite(input->yaw) && isfinite(input->yaw_vel) &&
                     isfinite(input->yaw_acc) && isfinite(input->pitch) &&
                     isfinite(input->pitch_vel) && isfinite(input->pitch_acc) &&
                     isfinite(input->yaw * radians_to_degrees) &&
                     isfinite(input->yaw_vel * radians_to_degrees) &&
                     isfinite(input->yaw_acc * radians_to_degrees) &&
                     isfinite(input->pitch * radians_to_degrees) &&
                     isfinite(input->pitch_vel * radians_to_degrees) &&
                     isfinite(input->pitch_acc * radians_to_degrees));
}

uint8_t VisionControlBuildShootCommand(const VisionControlInput_s *input,
                                       VisionControlShootCommand_s *command)
{
    if (command == NULL)
        return 0u;

    ResetShootCommand(command);

    if (!VisionControlInputIsUsable(input))
        return 0u;

    command->shoot_enabled = 1u;
    command->friction_enabled = 1u;
    if (input->mode == 2u)
    {
        command->load_mode = VISION_CONTROL_LOAD_BURSTFIRE;
        command->shoot_rate = 10.0f;
    }
    return 1u;
}

uint8_t VisionControlSelectShootCommand(const VisionControlInput_s *input,
                                        const VisionControlShootCommand_s *manual_command,
                                        VisionControlShootCommand_s *command)
{
    if (command == NULL)
        return 0u;

    if (VisionControlBuildShootCommand(input, command))
        return 1u;

    if (manual_command != NULL)
        *command = *manual_command;
    return 0u;
}

void VisionControlBuildRemoteShootCommand(int16_t dial,
                                          VisionControlShootCommand_s *command)
{
    if (command == NULL)
        return;

    ResetShootCommand(command);
    command->shoot_rate = 8.0f;
    command->friction_enabled = (uint8_t)(dial < -100);
    if (dial < -500)
        command->load_mode = VISION_CONTROL_LOAD_BURSTFIRE;
    EnableManualShootIfRequested(command);
}

void VisionControlBuildMouseKeyShootCommand(uint8_t load_cycle,
                                            uint8_t friction_cycle,
                                            VisionControlShootCommand_s *command)
{
    if (command == NULL)
        return;

    ResetShootCommand(command);
    command->shoot_rate = 8.0f;
    switch (load_cycle % 4u)
    {
    case 1u:
        command->load_mode = VISION_CONTROL_LOAD_SINGLE_BULLET;
        break;
    case 2u:
        command->load_mode = VISION_CONTROL_LOAD_THREE_BULLETS;
        break;
    case 3u:
        command->load_mode = VISION_CONTROL_LOAD_BURSTFIRE;
        break;
    default:
        break;
    }
    command->friction_enabled = (uint8_t)(friction_cycle % 2u != 0u);
    EnableManualShootIfRequested(command);
}
