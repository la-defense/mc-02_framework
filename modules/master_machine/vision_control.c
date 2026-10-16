/**
 * @file vision_control.c
 * @brief 检查视觉数据、换算云台角度，并计算这次使用的发射命令。
 *
 * 供 robot_cmd.c 调用，只计算命令，不直接控制电机或保存旧命令。
 * 参数和返回值见 vision_control.h。
 */
#include "vision_control.h"

#include <math.h>
#include <stddef.h>

/* 生成关闭命令，command 不能为 NULL。 */
static void ResetShootCommand(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = 0u;
    command->friction_enabled = 0u;
    command->load_mode = VISION_CONTROL_LOAD_STOP;
    command->shoot_rate = 0.0f;
}

/* 有摩擦轮或供弹请求时，设 shoot_enabled 为 1；command 不能为 NULL。 */
static void EnableManualShootIfRequested(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = (uint8_t)(command->friction_enabled ||
                                       command->load_mode != VISION_CONTROL_LOAD_STOP);
}

uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input)
{
    /* 直接调用云台计算，避免两处检查遗漏。 */
    VisionControlGimbalCommand_s command;
    return VisionControlBuildGimbalCommand(input, &command);
}

uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command)
{
    const float radians_to_degrees = 57.29577951308232f;
    if (command == NULL)
        return 0u;
    /* 先清零，避免检查失败后继续使用旧目标。 */
    *command = (VisionControlGimbalCommand_s){0};
    /* uint32_t 时间相减，在计时数字从 0 重新开始后仍能计算间隔。 */
    if (input == NULL || input->limits == NULL || !input->limits->configured || input->online == 0u ||
        (input->mode != 1u && input->mode != 2u) ||
        input->max_age_ms > VISION_CONTROL_MAX_AGE_MS ||
        (uint32_t)(input->now_ms - input->last_rx_ms) > input->max_age_ms)
        return 0u;

    const VisionControlLimits_s *limits = input->limits;
    if (!isfinite(limits->pitch_min_deg) || !isfinite(limits->pitch_max_deg) ||
        !isfinite(limits->pitch_zero_offset_deg) || limits->pitch_min_deg >= limits->pitch_max_deg ||
        !isfinite(limits->yaw_min_deg) || !isfinite(limits->yaw_max_deg) ||
        (limits->yaw_constraint != VISION_YAW_CONTINUOUS && limits->yaw_constraint != VISION_YAW_BOUNDED))
        return 0u;
    if (limits->yaw_constraint == VISION_YAW_BOUNDED &&
        limits->yaw_min_deg >= limits->yaw_max_deg)
        return 0u;
    if (!isfinite(input->yaw) || !isfinite(input->yaw_vel) || !isfinite(input->yaw_acc) ||
        !isfinite(input->pitch) || !isfinite(input->pitch_vel) || !isfinite(input->pitch_acc))
        return 0u;

    /* 先换算单位并补偿 pitch，再检查目标角度范围。 */
    const VisionControlGimbalCommand_s converted = {
        .yaw = input->yaw * radians_to_degrees,
        .pitch = input->pitch * radians_to_degrees + limits->pitch_zero_offset_deg,
        .yaw_vel = input->yaw_vel * radians_to_degrees,
        .yaw_acc = input->yaw_acc * radians_to_degrees,
        .pitch_vel = input->pitch_vel * radians_to_degrees,
        .pitch_acc = input->pitch_acc * radians_to_degrees,
    };
    /* 换算后再检查一次，防止过大的数变成无穷大。 */
    if (!isfinite(converted.yaw) || !isfinite(converted.pitch) ||
        !isfinite(converted.yaw_vel) || !isfinite(converted.yaw_acc) ||
        !isfinite(converted.pitch_vel) || !isfinite(converted.pitch_acc) ||
        converted.pitch < limits->pitch_min_deg || converted.pitch > limits->pitch_max_deg ||
        (limits->yaw_constraint == VISION_YAW_BOUNDED &&
         (converted.yaw < limits->yaw_min_deg || converted.yaw > limits->yaw_max_deg)))
        return 0u;
    *command = converted;
    return 1u;
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

    /* 没有手动命令时，保留前面生成的关闭命令。 */
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
