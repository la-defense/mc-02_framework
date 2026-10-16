/**
 * @file vision_control.c
 * @brief 检查视觉数据、换算云台角度，并计算这次使用的发射命令。
 *
 * robot_cmd.c 把视觉、遥控或键鼠数据交给这里，取回云台和发射机构将要使用的命令。
 * 视觉数据要先通过云台的全部检查，才能继续生成视觉发射命令。
 * 参数和返回值的说明放在 vision_control.h；这里只计算结果，不直接控制电机，也不保存上一次的命令。
 */
#include "vision_control.h"

#include <math.h>
#include <stddef.h>

/* 把发射、摩擦轮和供弹都设为关闭，每秒发射数量也清零。调用前要确认 command 不为 NULL。 */
static void ResetShootCommand(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = 0u;
    command->friction_enabled = 0u;
    command->load_mode = VISION_CONTROL_LOAD_STOP;
    command->shoot_rate = 0.0f;
}

/* 摩擦轮或供弹有一项需要工作，就打开 shoot_enabled。调用前要确认 command 不为 NULL。 */
static void EnableManualShootIfRequested(VisionControlShootCommand_s *command)
{
    command->shoot_enabled = (uint8_t)(command->friction_enabled ||
                                       command->load_mode != VISION_CONTROL_LOAD_STOP);
}

uint8_t VisionControlInputIsUsable(const VisionControlInput_s *input)
{
    /* 让云台计算函数实际算一次，生成发射命令前也能检查换算后的数字和角度范围。 */
    VisionControlGimbalCommand_s command;
    return VisionControlBuildGimbalCommand(input, &command);
}

uint8_t VisionControlBuildGimbalCommand(const VisionControlInput_s *input,
                                      VisionControlGimbalCommand_s *command)
{
    const float radians_to_degrees = 57.29577951308232f;
    if (command == NULL)
        return 0u;
    /* 先清零，后面任何一步失败都不会留下上一次的目标；全部通过后再填入新结果。 */
    *command = (VisionControlGimbalCommand_s){0};
    /* 先看通信、模式和接收时间。uint32_t 相减能处理计时数字到最大值后从 0 继续的情况。 */
    if (input == NULL || input->limits == NULL || !input->limits->configured || input->online == 0u ||
        (input->mode != 1u && input->mode != 2u) ||
        input->max_age_ms > VISION_CONTROL_MAX_AGE_MS ||
        (uint32_t)(input->now_ms - input->last_rx_ms) > input->max_age_ms)
        return 0u;

    /* 先检查设置里的数字，再按转动方式检查下限是否小于上限。 */
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

    /* 先把弧度换成度，再给 pitch 加补偿角度。比如 25 度加 5 度后，要检查的是 30 度。 */
    const VisionControlGimbalCommand_s converted = {
        .yaw = input->yaw * radians_to_degrees,
        .pitch = input->pitch * radians_to_degrees + limits->pitch_zero_offset_deg,
        .yaw_vel = input->yaw_vel * radians_to_degrees,
        .yaw_acc = input->yaw_acc * radians_to_degrees,
        .pitch_vel = input->pitch_vel * radians_to_degrees,
        .pitch_acc = input->pitch_acc * radians_to_degrees,
    };
    /* 换算可能让过大的数字变成无穷大，所以还要检查结果，以及补偿后的角度是否超出范围。 */
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
    /* mode 1 请求摩擦轮转动，但不送弹；mode 2 再请求每秒 10 发的连续供弹。 */
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

    /* 走到这里时已生成关闭命令。有这次算好的手动命令，就把它复制过来。 */
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
