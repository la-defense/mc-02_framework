/** @file vision_protocol_v2_test.c
 * @brief 用独立固定字节向量验证V2字段顺序、CRC和流式拼帧。
 * @note 仅链接生产编解码，测试在主机运行，不连接USB或机器人设备。
 */
#include "test_check.h"
#include "vision_protocol_v2.h"

#include <math.h>
#include <string.h>

static const uint8_t fixed_navigation_frame[] = {
    0x4du, 0x43u, 0x02u, 0x02u, 0x0du, 0x00u, 0x34u, 0x12u,
    0x04u, 0x03u, 0x02u, 0x01u, 0x03u, 0x00u, 0x00u, 0x80u,
    0x3fu, 0x00u, 0x00u, 0x00u, 0xbfu, 0x00u, 0x00u, 0x80u,
    0x3eu, 0xc2u, 0x9du
};

/* 这些帧的CRC由独立脚本按CRC16/MODBUS参数计算后固定保存。 */
static const uint8_t fixed_aim_frame[] = {
    0x4du, 0x43u, 0x02u, 0x01u, 0x19u, 0x00u, 0x02u, 0x01u,
    0x44u, 0x33u, 0x22u, 0x11u, 0x01u, 0x00u, 0x00u, 0x80u,
    0x3eu, 0x00u, 0x00u, 0x00u, 0xbfu, 0x00u, 0x00u, 0x40u,
    0x3fu, 0x00u, 0x00u, 0x00u, 0xbeu, 0x00u, 0x00u, 0x00u,
    0x3fu, 0x00u, 0x00u, 0x80u, 0xbfu, 0x1du, 0x10u
};

static const uint8_t fixed_posture_frame[] = {
    0x4du, 0x43u, 0x02u, 0x81u, 0x22u, 0x00u, 0x34u, 0x12u,
    0x88u, 0x77u, 0x66u, 0x55u, 0x03u, 0x01u, 0x00u, 0x00u,
    0x80u, 0x3fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x80u, 0x3eu, 0x00u, 0x00u, 0x00u, 0x3fu, 0x00u, 0x00u,
    0x00u, 0xbeu, 0x00u, 0x00u, 0x80u, 0xbeu, 0xebu, 0x7bu
};

static const uint8_t fixed_referee_frame[] = {
    0x4du, 0x43u, 0x02u, 0x82u, 0x24u, 0x00u, 0xcdu, 0xabu,
    0xccu, 0xbbu, 0xaau, 0x99u, 0x07u, 0x03u, 0x34u, 0x12u,
    0x45u, 0x23u, 0x56u, 0x34u, 0x00u, 0x00u, 0xf0u, 0x41u,
    0x05u, 0x67u, 0x45u, 0x78u, 0x56u, 0x89u, 0x67u, 0x0au,
    0x00u, 0x00u, 0xecu, 0x41u, 0x04u, 0x03u, 0x02u, 0x01u,
    0x14u, 0x13u, 0x12u, 0x11u, 0x24u, 0x23u, 0x22u, 0x21u,
    0x78u, 0x25u
};

typedef struct
{
    size_t count;
    VisionV2Frame_t last;
} Capture_t;

static void CaptureFrame(void *context, const VisionV2Frame_t *frame)
{
    Capture_t *capture = context;
    capture->count++;
    capture->last = *frame;
}

static int CompareFrame(uint8_t type, uint16_t sequence, uint32_t source_time_ms,
                        const uint8_t *payload, size_t payload_length,
                        const uint8_t *expected, size_t expected_length)
{
    uint8_t output[VISION_V2_MAX_FRAME];
    size_t output_length = 0u;
    memset(output, 0xa5, sizeof(output));
    TEST_CHECK(VisionV2EncodeFrame(type, sequence, source_time_ms, payload,
        payload_length, output, &output_length) == VISION_V2_OK);
    TEST_CHECK(output_length == expected_length);
    TEST_CHECK(memcmp(output, expected, expected_length) == 0);
    return 0;
}

static int TestAllFixedVectors(void)
{
    VisionV2AimCommand_t aim = {
        .mode = 1u, .yaw_rad = 0.25f, .yaw_rate_rad_s = -0.5f,
        .yaw_accel_rad_s2 = 0.75f, .pitch_rad = -0.125f,
        .pitch_rate_rad_s = 0.5f, .pitch_accel_rad_s2 = -1.0f,
    };
    uint8_t aim_payload[25] = {0};
    aim_payload[0] = aim.mode;
    const float aim_values[] = {aim.yaw_rad, aim.yaw_rate_rad_s, aim.yaw_accel_rad_s2,
                                aim.pitch_rad, aim.pitch_rate_rad_s, aim.pitch_accel_rad_s2};
    for (size_t i = 0u; i < sizeof(aim_values) / sizeof(aim_values[0]); ++i)
    {
        uint32_t bits;
        memcpy(&bits, &aim_values[i], sizeof(bits));
        for (size_t byte = 0u; byte < sizeof(bits); ++byte)
            aim_payload[1u + i * sizeof(bits) + byte] = (uint8_t)(bits >> (8u * byte));
    }
    TEST_CHECK(CompareFrame(VISION_V2_AIM_COMMAND, 0x0102u, 0x11223344u,
        aim_payload, sizeof(aim_payload), fixed_aim_frame, sizeof(fixed_aim_frame)) == 0);

    VisionV2PostureStatus_t posture = {
        .valid_flags = 3u, .robot_state = 1u,
        .quaternion_wxyz = {1.0f, 0.0f, 0.0f, 0.0f},
        .yaw_rad = 0.25f, .yaw_rate_rad_s = 0.5f,
        .pitch_rad = -0.125f, .pitch_rate_rad_s = -0.25f,
    };
    uint8_t posture_payload[34];
    TEST_CHECK(VisionV2EncodePosture(&posture, posture_payload,
        sizeof(posture_payload)) == VISION_V2_OK);
    TEST_CHECK(CompareFrame(VISION_V2_POSTURE_STATUS, 0x1234u, 0x55667788u,
        posture_payload, sizeof(posture_payload), fixed_posture_frame,
        sizeof(fixed_posture_frame)) == 0);

    VisionV2RefereeSummary_t summary = {
        .valid_flags = 7u, .robot_id = 3u,
        .current_hp = 0x1234u, .maximum_hp = 0x2345u,
        .chassis_power_limit_w = 0x3456u, .bullet_speed_limit_m_s = 30.0f,
        .power_output_flags = 5u, .buffer_energy_j = 0x4567u,
        .heat_17mm = 0x5678u, .heat_42mm = 0x6789u,
        .last_shot_frequency_hz = 10u, .last_shot_speed_m_s = 29.5f,
        .robot_state_source_ms = 0x01020304u,
        .power_heat_source_ms = 0x11121314u,
        .shoot_record_source_ms = 0x21222324u,
    };
    uint8_t referee_payload[36];
    TEST_CHECK(VisionV2EncodeRefereeSummary(&summary, referee_payload,
        sizeof(referee_payload)) == VISION_V2_OK);
    TEST_CHECK(CompareFrame(VISION_V2_REFEREE_SUMMARY, 0xabcdu, 0x99aabbccu,
        referee_payload, sizeof(referee_payload), fixed_referee_frame,
        sizeof(fixed_referee_frame)) == 0);
    return 0;
}

static int EncodeFixedVector(void)
{
    const uint8_t payload[] = {
        0x03u, 0x00u, 0x00u, 0x80u, 0x3fu, 0x00u, 0x00u,
        0x00u, 0xbfu, 0x00u, 0x00u, 0x80u, 0x3eu
    };
    uint8_t output[VISION_V2_MAX_FRAME];
    memset(output, 0xa5, sizeof(output));
    size_t length = 0u;
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_NAVIGATION_COMMAND, 0x1234u,
        0x01020304u, payload, sizeof(payload), output, &length) == VISION_V2_OK);
    TEST_CHECK(length == sizeof(fixed_navigation_frame));
    TEST_CHECK(memcmp(output, fixed_navigation_frame, length) == 0);
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_NAVIGATION_COMMAND, 0x1234u,
        0x01020304u, payload, sizeof(payload) - 1u, output, &length) == VISION_V2_ERR_LENGTH);
    TEST_CHECK(length == sizeof(fixed_navigation_frame));
    TEST_CHECK(memcmp(output, fixed_navigation_frame, length) == 0);
    return 0;
}

static int TestAllSplitPoints(void)
{
    for (size_t split = 1u; split < sizeof(fixed_navigation_frame); ++split)
    {
        VisionV2Stream_t stream = {0};
        Capture_t capture = {0};
        uint32_t errors = 0u;
        TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, split,
            7u, 10u, 3u, 7u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
        TEST_CHECK(capture.count == 0u && errors == 0u);
        TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[split],
            sizeof(fixed_navigation_frame) - split, 8u, 11u, 3u, 8u,
            CaptureFrame, &capture, &errors) == VISION_V2_OK);
        TEST_CHECK(capture.count == 1u && errors == 0u);
        TEST_CHECK(capture.last.type == VISION_V2_NAVIGATION_COMMAND &&
            capture.last.sequence == 0x1234u &&
            capture.last.source_time_ms == 0x01020304u &&
            capture.last.received_time_ms == 7u && capture.last.generation == 3u);
        VisionV2NavigationCommand_t nav = {0};
        TEST_CHECK(VisionV2DecodeNavigation(capture.last.payload,
            capture.last.payload_length, &nav) == VISION_V2_OK);
        TEST_CHECK(nav.valid_flags == 3u && nav.vx_m_s == 1.0f &&
            nav.vy_m_s == -0.5f && nav.yaw_rate_rad_s == 0.25f);
    }
    return 0;
}

static int TestNoiseMultiFrameAndReset(void)
{
    VisionV2Stream_t stream = {0};
    Capture_t capture = {0};
    uint8_t bytes[sizeof(fixed_navigation_frame) * 2u + 3u];
    bytes[0] = 0x00u;
    bytes[1] = 0x4du;
    bytes[2] = 0x00u;
    memcpy(&bytes[3], fixed_navigation_frame, sizeof(fixed_navigation_frame));
    memcpy(&bytes[3u + sizeof(fixed_navigation_frame)], fixed_navigation_frame,
           sizeof(fixed_navigation_frame));
    uint32_t errors = 0u;
    TEST_CHECK(VisionV2StreamFeed(&stream, bytes, sizeof(bytes), 20u, 1u, 4u,
        20u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 2u && errors == 0u);

    VisionV2StreamReset(&stream);
    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 5u, 30u,
        8u, 4u, 30u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(stream.used == 5u);
    TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[5],
        sizeof(fixed_navigation_frame) - 5u, 31u, 10u, 4u, 31u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 2u);
    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 5u, 40u,
        11u, 4u, 40u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(stream.used == 5u);
    TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[5],
        sizeof(fixed_navigation_frame) - 5u, 41u, 12u, 5u, 41u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 2u && stream.used == 0u);
    return 0;
}

static int TestDecodeAndEncodeBoundaries(void)
{
    size_t length = 0u;
    TEST_CHECK(VisionV2GetPayloadSize(VISION_V2_AIM_COMMAND, &length) == VISION_V2_OK && length == 25u);
    TEST_CHECK(VisionV2GetPayloadSize(VISION_V2_NAVIGATION_COMMAND, &length) == VISION_V2_OK && length == 13u);
    TEST_CHECK(VisionV2GetPayloadSize(VISION_V2_POSTURE_STATUS, &length) == VISION_V2_OK && length == 34u);
    TEST_CHECK(VisionV2GetPayloadSize(VISION_V2_REFEREE_SUMMARY, &length) == VISION_V2_OK && length == 36u);
    length = 0x55u;
    TEST_CHECK(VisionV2GetPayloadSize(0xffu, &length) == VISION_V2_ERR_TYPE &&
        length == 0x55u);
    TEST_CHECK(VisionV2GetPayloadSize(VISION_V2_AIM_COMMAND, NULL) ==
        VISION_V2_ERR_ARGUMENT);

    uint8_t bad_frame[sizeof(fixed_navigation_frame)];
    memcpy(bad_frame, fixed_navigation_frame, sizeof(bad_frame));
    bad_frame[sizeof(bad_frame) - 1u] ^= 0x80u;
    VisionV2Stream_t stream = {0};
    Capture_t capture = {0};
    uint32_t errors = 0u;
    TEST_CHECK(VisionV2StreamFeed(&stream, bad_frame, sizeof(bad_frame), 50u,
        1u, 1u, 50u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 0u && errors == 1u);

    uint8_t payload[13] = {0};
    payload[0] = 0x80u;
    VisionV2NavigationCommand_t nav = {0x5au, 1.0f, 2.0f, 3.0f};
    TEST_CHECK(VisionV2DecodeNavigation(payload, sizeof(payload), &nav) == VISION_V2_ERR_VALUE);
    TEST_CHECK(nav.valid_flags == 0x5au && nav.vx_m_s == 1.0f);

    VisionV2PostureStatus_t posture = {0};
    uint8_t posture_payload[34];
    memset(posture_payload, 0x69, sizeof(posture_payload));
    posture.yaw_rad = NAN;
    TEST_CHECK(VisionV2EncodePosture(&posture, posture_payload,
        sizeof(posture_payload)) == VISION_V2_ERR_VALUE);
    TEST_CHECK(posture_payload[0] == 0x69u && posture_payload[33] == 0x69u);

    const uint8_t infinite_navigation[] = {
        VISION_V2_NAV_TRANSLATION_VALID, 0x00u, 0x00u, 0x80u, 0x7fu,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
    };
    nav = (VisionV2NavigationCommand_t){0x5au, 1.0f, 2.0f, 3.0f};
    TEST_CHECK(VisionV2DecodeNavigation(infinite_navigation,
        sizeof(infinite_navigation), &nav) == VISION_V2_ERR_VALUE);
    TEST_CHECK(nav.valid_flags == 0x5au && nav.vx_m_s == 1.0f);

    uint8_t invalid_aim[25] = {2u};
    VisionV2AimCommand_t previous_aim = {.mode = 0u, .yaw_rad = 1.0f};
    TEST_CHECK(VisionV2DecodeAim(invalid_aim, sizeof(invalid_aim), &previous_aim) ==
        VISION_V2_ERR_VALUE);
    TEST_CHECK(previous_aim.mode == 0u && previous_aim.yaw_rad == 1.0f);

    uint8_t encoded_referee[36];
    memset(encoded_referee, 0x3cu, sizeof(encoded_referee));
    VisionV2RefereeSummary_t invalid_summary = {.valid_flags = 0x80u};
    TEST_CHECK(VisionV2EncodeRefereeSummary(&invalid_summary, encoded_referee,
        sizeof(encoded_referee)) == VISION_V2_ERR_VALUE);
    TEST_CHECK(encoded_referee[0] == 0x3cu && encoded_referee[35] == 0x3cu);

    uint8_t bad_version[sizeof(fixed_navigation_frame)];
    memcpy(bad_version, fixed_navigation_frame, sizeof(bad_version));
    bad_version[2] = 3u;
    memset(&stream, 0, sizeof(stream));
    errors = 0u;
    TEST_CHECK(VisionV2StreamFeed(&stream, bad_version, sizeof(bad_version), 60u,
        1u, 1u, 60u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(errors == 1u && capture.count == 0u);
    const size_t old_used = stream.used;
    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 0u, 70u,
        2u, 1u, 70u, CaptureFrame, &capture, &errors) == VISION_V2_ERR_ARGUMENT);
    TEST_CHECK(stream.used == old_used);
    return 0;
}

static int TestStreamDiscontinuityAndExpiry(void)
{
    VisionV2Stream_t stream = {0};
    Capture_t capture = {0};
    uint32_t errors = 0u;
    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 8u, 10u,
        5u, 1u, 10u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(stream.used == 8u);
    TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[8],
        sizeof(fixed_navigation_frame) - 8u, 11u, 7u, 1u, 11u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 0u);

    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 8u, 20u,
        8u, 1u, 20u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[8],
        sizeof(fixed_navigation_frame) - 8u, 21u, 9u, 2u, 21u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 0u);

    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame, 8u, 30u,
        10u, 2u, 30u, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(VisionV2StreamFeed(&stream, &fixed_navigation_frame[8],
        sizeof(fixed_navigation_frame) - 8u, 81u, 11u, 2u, 81u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 0u);

    TEST_CHECK(VisionV2StreamFeed(&stream, fixed_navigation_frame,
        sizeof(fixed_navigation_frame), 82u, 12u, 2u, 82u,
        CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(capture.count == 1u);
    return 0;
}

int main(void)
{
    TEST_CHECK(TestAllFixedVectors() == 0);
    TEST_CHECK(EncodeFixedVector() == 0);
    TEST_CHECK(TestAllSplitPoints() == 0);
    TEST_CHECK(TestNoiseMultiFrameAndReset() == 0);
    TEST_CHECK(TestDecodeAndEncodeBoundaries() == 0);
    TEST_CHECK(TestStreamDiscontinuityAndExpiry() == 0);
    return 0;
}
