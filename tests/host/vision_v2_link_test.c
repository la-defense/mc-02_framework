/** @file vision_v2_link_test.c
 * @brief 通过生产MasterProcess与消息中心验证V2接收发布及状态回传。
 * @note USB端点由本机回调模型替代；不连接设备，所有检查在Release下生效。
 */
#include "test_check.h"
#include "master_process.h"
#include "daemon.h"
#include "message_center.h"
#include "referee_snapshot.h"
#include "robot_topics.h"
#include "rm_referee.h"
#include "usb_transport.h"
#include "vision_protocol_v2.h"

#include <string.h>

static uint32_t now_ms;
static USBTransportConfig_t usb_config;
static uint32_t current_generation = 1u;
static uint32_t valid_command_reports;
static uint32_t daemon_reloads;
static Daemon_Init_Config_s daemon_config;
static DaemonInstance daemon_instance;
static uint8_t tx_frame[VISION_V2_MAX_FRAME];
static size_t tx_length;
static uint32_t tx_count;
static RefereeSnapshot_t referee_snapshot;

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

USBStatus_t USBTransportInit(const USBTransportConfig_t *config)
{
    if (config == NULL)
        return USB_BAD_ARGUMENT;
    usb_config = *config;
    return USB_OK;
}

USBStatus_t USBTransportSubmit(const uint8_t *data, size_t length, uint32_t *id)
{
    if (data == NULL || id == NULL || length > sizeof(tx_frame))
        return USB_BAD_ARGUMENT;
    memcpy(tx_frame, data, length);
    tx_length = length;
    ++tx_count;
    *id = tx_count;
    return USB_ACCEPTED;
}

uint8_t USBTransportGenerationIsCurrent(uint32_t generation)
{
    return (uint8_t)(generation == current_generation);
}

void USBTransportReportValidCommand(uint32_t generation, uint32_t received_ms)
{
    if (generation == current_generation &&
        (uint32_t)(now_ms - received_ms) <= USB_SOURCE_MAX_AGE_MS)
        ++valid_command_reports;
}

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (config == NULL)
        return NULL;
    daemon_config = *config;
    return &daemon_instance;
}

uint8_t DaemonUnregister(DaemonInstance *instance)
{
    return (uint8_t)(instance == &daemon_instance);
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance == &daemon_instance)
        ++daemon_reloads;
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return (uint8_t)(instance == &daemon_instance);
}

RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output)
{
    if (output == NULL)
        return REFEREE_ERROR_ARGUMENT;
    *output = referee_snapshot;
    return REFEREE_OK;
}

typedef struct
{
    uint32_t count;
    VisionV2Frame_t last;
} FrameCapture_t;

static void CaptureFrame(void *context, const VisionV2Frame_t *frame)
{
    FrameCapture_t *capture = context;
    ++capture->count;
    capture->last = *frame;
}

static int CheckLastTx(uint8_t expected_type, uint16_t expected_sequence)
{
    VisionV2Stream_t stream = {0};
    FrameCapture_t capture = {0};
    uint32_t errors = 0u;
    TEST_CHECK(tx_length > 0u);
    TEST_CHECK(VisionV2StreamFeed(&stream, tx_frame, tx_length, now_ms, 1u,
        1u, now_ms, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(errors == 0u && capture.count == 1u);
    TEST_CHECK(capture.last.type == expected_type &&
        capture.last.sequence == expected_sequence &&
        capture.last.source_time_ms == now_ms);
    return 0;
}

static void InjectFrame(const uint8_t *data, size_t length, uint32_t received_ms,
                        uint32_t sequence, uint32_t generation)
{
    const USBRxBlock_t block = {
        .data = data,
        .length = length,
        .received_ms = received_ms,
        .sequence = sequence,
        .generation = generation,
    };
    usb_config.receive(usb_config.context, &block);
}

static int CheckInboundTopics(void)
{
    MessageSubscriber_t *aim_subscriber = MessageSubscriberRegister(
        ROBOT_TOPIC_VISION_AIM, ROBOT_TOPIC_ENDPOINT_COMMAND);
    MessageSubscriber_t *navigation_subscriber = MessageSubscriberRegister(
        ROBOT_TOPIC_VISION_NAVIGATION, ROBOT_TOPIC_ENDPOINT_COMMAND);
    TEST_CHECK(aim_subscriber != NULL && navigation_subscriber != NULL);

    VisionV2AimCommand_t aim = {
        .mode = 1u, .yaw_rad = 0.25f, .yaw_rate_rad_s = -0.5f,
        .yaw_accel_rad_s2 = 0.75f, .pitch_rad = -0.125f,
        .pitch_rate_rad_s = 0.25f, .pitch_accel_rad_s2 = -0.375f,
    };
    uint8_t aim_payload[25];
    memcpy(aim_payload, &aim, 1u);
    const float aim_values[] = {aim.yaw_rad, aim.yaw_rate_rad_s, aim.yaw_accel_rad_s2,
                                aim.pitch_rad, aim.pitch_rate_rad_s, aim.pitch_accel_rad_s2};
    for (size_t i = 0u; i < sizeof(aim_values) / sizeof(aim_values[0]); ++i)
    {
        uint32_t bits;
        memcpy(&bits, &aim_values[i], sizeof(bits));
        for (size_t byte = 0u; byte < sizeof(bits); ++byte)
            aim_payload[1u + i * sizeof(bits) + byte] = (uint8_t)(bits >> (8u * byte));
    }

    uint8_t frame[VISION_V2_MAX_FRAME];
    size_t frame_length = 0u;
    now_ms = 0u;
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_AIM_COMMAND, 0xfffeu, 0xfffffff0u,
        aim_payload, sizeof(aim_payload), frame, &frame_length) == VISION_V2_OK);
    InjectFrame(frame, 7u, now_ms, 1u, current_generation);
    now_ms = 3u;
    InjectFrame(&frame[7], frame_length - 7u, now_ms, 2u, current_generation);
    TEST_CHECK(daemon_reloads == 1u && valid_command_reports == 1u);
    Vision_Status_t status;
    VisionGetStatus(&status);
    TEST_CHECK(status.online != 0u && status.rx_count == 1u && status.last_rx_ms == 0u);

    VisionV2AimCommand_t actual_aim = {0};
    MessageFreshness_t freshness = {0};
    TEST_CHECK(MessageReadSnapshot(aim_subscriber, &actual_aim, sizeof(actual_aim),
        now_ms, &freshness) == MESSAGE_OK);
    TEST_CHECK(actual_aim.mode == aim.mode && actual_aim.yaw_rad == aim.yaw_rad &&
        actual_aim.pitch_rate_rad_s == aim.pitch_rate_rad_s);
    TEST_CHECK(freshness.source_time_ms == 0xfffffff0u &&
        freshness.received_time_ms == 0u && freshness.published_time_ms == 3u &&
        freshness.source_clock == MESSAGE_CLOCK_REMOTE_RAW);

    // 同一序号和过期瞄准均不能覆盖最近一次合法数据或刷新整链路时间。
    const uint32_t accepted_rx = status.rx_count;
    now_ms = 4u;
    InjectFrame(frame, frame_length, 4u, 3u, current_generation);
    now_ms = VISION_V2_AIM_MAX_AGE_MS + 10u;
    InjectFrame(frame, frame_length, now_ms - VISION_V2_AIM_MAX_AGE_MS - 1u,
        4u, current_generation);
    VisionGetStatus(&status);
    TEST_CHECK(status.rx_count == accepted_rx && status.last_rx_ms == 0u);
    TEST_CHECK(MessageReadSnapshot(aim_subscriber, &actual_aim, sizeof(actual_aim),
        now_ms, &freshness) == MESSAGE_ERR_EXPIRED);

    // 导航使用独立序号和期限；中间丢失的序号无需连续，重复帧不重复发布。
    VisionV2NavigationCommand_t navigation = {
        .valid_flags = VISION_V2_NAV_TRANSLATION_VALID | VISION_V2_NAV_YAW_RATE_VALID,
        .vx_m_s = 1.25f, .vy_m_s = -0.5f, .yaw_rate_rad_s = 2.0f,
    };
    uint8_t navigation_payload[13] = {3u};
    const float navigation_values[] = {navigation.vx_m_s, navigation.vy_m_s,
                                       navigation.yaw_rate_rad_s};
    for (size_t i = 0u; i < sizeof(navigation_values) / sizeof(navigation_values[0]); ++i)
    {
        uint32_t bits;
        memcpy(&bits, &navigation_values[i], sizeof(bits));
        for (size_t byte = 0u; byte < sizeof(bits); ++byte)
            navigation_payload[1u + i * sizeof(bits) + byte] = (uint8_t)(bits >> (8u * byte));
    }
    ++now_ms;
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_NAVIGATION_COMMAND, 0u, 0u,
        navigation_payload, sizeof(navigation_payload), frame, &frame_length) == VISION_V2_OK);
    InjectFrame(frame, frame_length, now_ms, 5u, current_generation);
    TEST_CHECK(MessageReadSnapshot(navigation_subscriber, &navigation, sizeof(navigation),
        now_ms, &freshness) == MESSAGE_OK);
    TEST_CHECK(navigation.vx_m_s == 1.25f && navigation.vy_m_s == -0.5f &&
        navigation.yaw_rate_rad_s == 2.0f && freshness.sequence == 1u);
    InjectFrame(frame, frame_length, now_ms, 6u, current_generation);
    MessageTopicStats_t stats;
    TEST_CHECK(MessageGetTopicStats(ROBOT_TOPIC_VISION_NAVIGATION, &stats) == MESSAGE_OK &&
        stats.published == 1u);

    // 新连接的首帧可从任意序号恢复，并先撤销另一类型的旧连接快照。
    current_generation = 2u;
    ++now_ms;
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_AIM_COMMAND, 0u, 1u,
        aim_payload, sizeof(aim_payload), frame, &frame_length) == VISION_V2_OK);
    InjectFrame(frame, frame_length, now_ms, 7u, current_generation);
    TEST_CHECK(MessageGetTopicStats(ROBOT_TOPIC_VISION_NAVIGATION, &stats) == MESSAGE_OK &&
        stats.published == 2u);
    TEST_CHECK(MessageReadSnapshot(navigation_subscriber, &navigation, sizeof(navigation),
        now_ms, &freshness) == MESSAGE_OK && navigation.valid_flags == 0u &&
        freshness.received_time_ms == now_ms);
    InjectFrame(frame, frame_length, now_ms, 8u, 1u);
    TEST_CHECK(MessageGetTopicStats(ROBOT_TOPIC_VISION_NAVIGATION, &stats) == MESSAGE_OK &&
        stats.published == 2u);
    return 0;
}

static int CheckOutboundReports(void)
{
    VisionV2PostureStatus_t posture = {
        .valid_flags = VISION_V2_POSTURE_ORIENTATION_VALID | VISION_V2_POSTURE_RATE_VALID,
        .robot_state = 1u,
        .quaternion_wxyz = {1.0f, 0.0f, 0.0f, 0.0f},
        .yaw_rad = 0.25f,
        .yaw_rate_rad_s = 0.5f,
        .pitch_rad = -0.125f,
        .pitch_rate_rad_s = -0.25f,
    };
    referee_snapshot.command[REF_CMD_ROBOT_STATE].state = REFEREE_DATA_FRESH;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].source_rx_ms = 10u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id = 3u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.current_hp = 100u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.maximum_hp = 200u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.chassis_power_limit_w = 80u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.bullet_speed_limit_mps = 30.0f;
    referee_snapshot.command[REF_CMD_POWER_HEAT].state = REFEREE_DATA_FRESH;
    referee_snapshot.command[REF_CMD_POWER_HEAT].source_rx_ms = 20u;
    referee_snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.buffer_energy_j = 50u;
    referee_snapshot.command[REF_CMD_SHOOT_DATA].state = REFEREE_DATA_EVENT_RECORDED;
    referee_snapshot.command[REF_CMD_SHOOT_DATA].source_rx_ms = 30u;
    referee_snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.frequency_hz = 10u;
    referee_snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.initial_speed_mps = 29.5f;

    now_ms = 100u;
    VisionUpdateTx(1u, posture.quaternion_wxyz[0], posture.quaternion_wxyz[1],
        posture.quaternion_wxyz[2], posture.quaternion_wxyz[3], posture.yaw_rad,
        posture.yaw_rate_rad_s, posture.pitch_rad, posture.pitch_rate_rad_s, 30.0f, 2u);
    VisionSend();
    TEST_CHECK(tx_count == 1u && CheckLastTx(VISION_V2_POSTURE_STATUS, 0u) == 0);

    now_ms += 50u;
    VisionSend();
    TEST_CHECK(tx_count == 2u && CheckLastTx(VISION_V2_REFEREE_SUMMARY, 1u) == 0);
    TEST_CHECK(valid_command_reports == 3u);
    return 0;
}

int main(void)
{
    TEST_CHECK(RobotTopicsInit() != 0u);
    TEST_CHECK(VisionInit(NULL) != NULL);
    TEST_CHECK(usb_config.receive != NULL && daemon_config.reload_count == 30u);
    TEST_CHECK(CheckInboundTopics() == 0);
    TEST_CHECK(CheckOutboundReports() == 0);
    return 0;
}
