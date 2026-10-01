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
        .burstfire_enabled = 1u,
        .shoot_rate = 10.0f,
    };

    CHECK(VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.burstfire_enabled == 1u && command.shoot_rate == 10.0f);

    input.mode = 1u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 1u && command.friction_enabled == 1u);
    CHECK(command.burstfire_enabled == 0u && command.shoot_rate == 0.0f);

    input.mode = 0u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.burstfire_enabled == 0u && command.shoot_rate == 0.0f);

    input.mode = 2u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    input.now_ms = 151u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.burstfire_enabled == 0u && command.shoot_rate == 0.0f);

    input.now_ms = 120u;
    CHECK(VisionControlBuildShootCommand(&input, &command));
    input.online = 0u;
    CHECK(!VisionControlBuildShootCommand(&input, &command));
    CHECK(command.shoot_enabled == 0u && command.friction_enabled == 0u);
    CHECK(command.burstfire_enabled == 0u && command.shoot_rate == 0.0f);
    return 0;
}

int main(void)
{
    CHECK(TestProtocol() == 0);
    CHECK(TestVisionControlValidity() == 0);
    CHECK(TestVisionControlRevokesStaleShootCommand() == 0);
    puts("MC-02 SP protocol and vision freshness checks passed");
    return 0;
}
