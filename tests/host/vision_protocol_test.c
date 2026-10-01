#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "vision_control.h"
#include "vision_protocol.h"

#define CHECK(condition)                                      \
    do                                                        \
    {                                                         \
        if (!(condition))                                     \
        {                                                     \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition); \
            return 1;                                         \
        }                                                     \
    } while (0)

static void MakeCommandFrame(Vision_Recv_s *frame, uint8_t mode)
{
    memset(frame, 0, sizeof(*frame));
    frame->head[0] = 'S';
    frame->head[1] = 'P';
    frame->mode = mode;
    frame->yaw = 0.25f;
    frame->pitch = -0.1f;
    frame->crc16 = VisionProtocolCRC16((const uint8_t *)frame, sizeof(*frame) - 2u);
}

static int TestProtocol(void)
{
    static const uint8_t check_vector[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(sizeof(Vision_Recv_s) == 29u);
    CHECK(sizeof(Vision_Send_s) == 43u);
    CHECK(VisionProtocolCRC16(check_vector, sizeof(check_vector)) == 0x6f91u);

    Vision_Recv_s expected;
    Vision_Recv_s received;
    uint32_t crc_errors = 0u;
    MakeCommandFrame(&expected, 1u);
    CHECK(VisionProtocolFindCommandFrame((const uint8_t *)&expected, sizeof(expected),
                                         &received, &crc_errors));
    CHECK(crc_errors == 0u);
    CHECK(received.mode == 1u && received.yaw == expected.yaw);

    uint8_t prefixed[sizeof(expected) + 3u] = {'x', 'y', 'z'};
    memcpy(prefixed + 3u, &expected, sizeof(expected));
    CHECK(VisionProtocolFindCommandFrame(prefixed, sizeof(prefixed), &received, &crc_errors));
    CHECK(received.mode == 1u && crc_errors == 0u);

    Vision_Recv_s corrupt = expected;
    corrupt.yaw += 1.0f;
    CHECK(!VisionProtocolFindCommandFrame((const uint8_t *)&corrupt, sizeof(corrupt),
                                          &received, &crc_errors));
    CHECK(crc_errors == 1u);
    CHECK(!VisionProtocolFindCommandFrame((const uint8_t *)&expected, 12u,
                                          &received, &crc_errors));
    CHECK(crc_errors == 0u);

    uint8_t recovery[sizeof(expected) * 2u];
    memcpy(recovery, &corrupt, sizeof(corrupt));
    memcpy(recovery + sizeof(corrupt), &expected, sizeof(expected));
    CHECK(VisionProtocolFindCommandFrame(recovery, sizeof(recovery), &received, &crc_errors));
    CHECK(crc_errors == 1u && received.mode == 1u);
    return 0;
}

static int TestVisionControlValidity(void)
{
    VisionControlInput_s input = {
        .online = 1u,
        .mode = 1u,
        .last_rx_ms = 100u,
        .now_ms = 150u,
        .max_age_ms = 50u,
        .yaw = 0.1f,
        .yaw_vel = 0.2f,
        .yaw_acc = 0.3f,
        .pitch = -0.1f,
        .pitch_vel = -0.2f,
        .pitch_acc = -0.3f,
    };
    CHECK(VisionControlInputIsUsable(&input));

    input.mode = 2u;
    CHECK(VisionControlInputIsUsable(&input));
    input.mode = 0u;
    CHECK(!VisionControlInputIsUsable(&input));
    input.mode = 3u;
    CHECK(!VisionControlInputIsUsable(&input));

    input.mode = 1u;
    input.online = 0u;
    CHECK(!VisionControlInputIsUsable(&input));
    input.online = 1u;
    input.now_ms = 151u;
    CHECK(!VisionControlInputIsUsable(&input));

    input.last_rx_ms = UINT32_MAX - 15u;
    input.now_ms = 20u;
    CHECK(VisionControlInputIsUsable(&input)); /* Tick counter wrapped; elapsed time is 36 ms. */

    input.yaw = NAN;
    CHECK(!VisionControlInputIsUsable(&input));
    input.yaw = INFINITY;
    CHECK(!VisionControlInputIsUsable(&input));
    input.yaw = 0.1f;
    input.pitch_acc = NAN;
    CHECK(!VisionControlInputIsUsable(&input));
    return 0;
}

static int TestVisionControlRevokesStaleShootCommand(void)
{
    VisionControlInput_s input = {
        .online = 1u,
        .mode = 2u,
        .last_rx_ms = 100u,
        .now_ms = 120u,
        .max_age_ms = 50u,
        .yaw = 0.1f,
        .yaw_vel = 0.2f,
        .yaw_acc = 0.3f,
        .pitch = -0.1f,
        .pitch_vel = -0.2f,
        .pitch_acc = -0.3f,
    };
    VisionControlShootCommand_s command = {
        .shoot_enabled = 1u,
        .friction_enabled = 1u,
        .load_mode = VISION_CONTROL_LOAD_BURSTFIRE,
        .shoot_rate = 10.0f,
    };

    CHECK(VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && command.shoot_rate == 10.0f);

    input.mode = 1u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 0.0f);

    input.mode = 0u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 0.0f);

    input.mode = 2u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    input.now_ms = 151u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 0.0f);

    input.now_ms = 120u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    input.online = 0u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 0.0f);
    return 0;
}

static int TestVisionControlPreservesManualShootCommand(void)
{
    VisionControlInput_s input = {
        .online = 1u,
        .mode = 2u,
        .last_rx_ms = 100u,
        .now_ms = 120u,
        .max_age_ms = 50u,
        .yaw = 0.1f,
        .yaw_vel = 0.2f,
        .yaw_acc = 0.3f,
        .pitch = -0.1f,
        .pitch_vel = -0.2f,
        .pitch_acc = -0.3f,
    };
    VisionControlShootCommand_s rc_manual_command;
    VisionControlShootCommand_s keyboard_manual_command;
    VisionControlShootCommand_s selected = {0};

    VisionControlBuildRemoteShootCommand(-501, &rc_manual_command);
    VisionControlBuildMouseKeyShootCommand(2u, 0u, &keyboard_manual_command);

    /* A valid vision command still has priority over the manual source. */
    CHECK(VisionControlSelectShootCommand(&input, &rc_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 1u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && selected.shoot_rate == 10.0f);

    input.mode = 1u;
    CHECK(VisionControlSelectShootCommand(&input, &rc_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 1u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_STOP && selected.shoot_rate == 0.0f);

    /* Keep mode 2 -> 1 -> 0 on one RC command to test handoff order explicitly. */
    input.mode = 0u;
    CHECK(!VisionControlSelectShootCommand(&input, &rc_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 1u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && selected.shoot_rate == 8.0f);

    /* A keyboard burst command also returns from vision rate 10 to manual rate 8. */
    input.mode = 2u;
    CHECK(VisionControlSelectShootCommand(&input, &keyboard_manual_command, &selected));
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && selected.shoot_rate == 10.0f);
    input.mode = 0u;
    CHECK(!VisionControlSelectShootCommand(&input, &keyboard_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 0u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_THREE_BULLETS && selected.shoot_rate == 8.0f);

    input.mode = 1u;
    input.now_ms = 151u;
    CHECK(!VisionControlSelectShootCommand(&input, &keyboard_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 0u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_THREE_BULLETS && selected.shoot_rate == 8.0f);

    input.now_ms = 120u;
    input.online = 0u;
    CHECK(!VisionControlSelectShootCommand(&input, &keyboard_manual_command, &selected));
    CHECK(selected.shoot_enabled == 1u && selected.friction_enabled == 0u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_THREE_BULLETS && selected.shoot_rate == 8.0f);

    /* Without a selected manual source, invalid vision clears the old command. */
    CHECK(!VisionControlSelectShootCommand(&input, NULL, &selected));
    CHECK(selected.shoot_enabled == 0u && selected.friction_enabled == 0u);
    CHECK(selected.load_mode == VISION_CONTROL_LOAD_STOP && selected.shoot_rate == 0.0f);
    return 0;
}

static int TestManualShootCommandMapping(void)
{
    VisionControlShootCommand_s command;

    VisionControlBuildRemoteShootCommand(-100, &command);
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 8.0f);

    VisionControlBuildRemoteShootCommand(-101, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 8.0f);

    VisionControlBuildRemoteShootCommand(-500, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 8.0f);

    VisionControlBuildRemoteShootCommand(-501, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && command.shoot_rate == 8.0f);

    VisionControlBuildMouseKeyShootCommand(0u, 0u, &command);
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_STOP && command.shoot_rate == 8.0f);

    VisionControlBuildMouseKeyShootCommand(1u, 0u, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_SINGLE_BULLET && command.shoot_rate == 8.0f);

    VisionControlBuildMouseKeyShootCommand(2u, 1u, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_THREE_BULLETS && command.shoot_rate == 8.0f);

    VisionControlBuildMouseKeyShootCommand(3u, 0u, &command);
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 0u);
    CHECK(command.load_mode == VISION_CONTROL_LOAD_BURSTFIRE && command.shoot_rate == 8.0f);
    return 0;
}

int main(void)
{
    CHECK(TestProtocol() == 0);
    CHECK(TestVisionControlValidity() == 0);
    CHECK(TestVisionControlRevokesStaleShootCommand() == 0);
    CHECK(TestVisionControlPreservesManualShootCommand() == 0);
    CHECK(TestManualShootCommandMapping() == 0);
    puts("MC-02 SP protocol and vision freshness checks passed");
    return 0;
}
