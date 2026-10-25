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
#include "time_mapping.h"
#include "usb_transport.h"
#include "vision_protocol_v2.h"

#include <math.h>
#include <string.h>

static uint32_t now_ms;
static USBTransportConfig_t usb_config;
static uint32_t current_generation = 1u;
static uint32_t valid_command_reports;
static uint32_t daemon_reloads;
static USBStatus_t next_submit_status = USB_ACCEPTED;
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

USBStatus_t USBTransportGetState(USBTransportState_t *output)
{
    if (output == NULL)
        return USB_BAD_ARGUMENT;
    memset(output, 0, sizeof(*output));
    output->phase = USB_PHASE_READY;
    output->generation = current_generation;
    output->initialized = 1u;
    return USB_OK;
}

USBStatus_t USBTransportSubmit(const uint8_t *data, size_t length, uint32_t *id)
{
    if (data == NULL || id == NULL || length > sizeof(tx_frame))
        return USB_BAD_ARGUMENT;
    const USBStatus_t result = next_submit_status;
    next_submit_status = USB_ACCEPTED;
    if (result != USB_ACCEPTED)
        return result;
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

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
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

DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{
    if (instance != &daemon_instance)
        return DAEMON_ERR_NOT_REGISTERED;
    if ((flags & DAEMON_REPORT_TRANSPORT_VALID) == 0u)
        return DAEMON_OK;
    ++daemon_reloads;
    (void)received_ms;
    return DAEMON_OK;
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

static int DecodeLastTx(VisionV2Frame_t *frame)
{
    VisionV2Stream_t stream = {0};
    FrameCapture_t capture = {0};
    uint32_t errors = 0u;
    TEST_CHECK(frame != NULL && tx_length > 0u);
    TEST_CHECK(VisionV2StreamFeed(&stream, tx_frame, tx_length, now_ms, 1u,
        1u, now_ms, CaptureFrame, &capture, &errors) == VISION_V2_OK);
    TEST_CHECK(errors == 0u && capture.count == 1u);
    *frame = capture.last;
    return 0;
}

static int InjectTimeReply(uint32_t request_id, uint32_t t2_ms,
                           uint32_t t3_ms, uint32_t t4_ms,
                           uint32_t block_sequence, uint32_t generation)
{
    VisionV2TimeSyncReply_t reply = {
        .request_id = request_id,
        .received_time_ms = t2_ms,
    };
    uint8_t payload[8];
    uint8_t frame[VISION_V2_MAX_FRAME];
    size_t frame_length = 0u;
    TEST_CHECK(VisionV2EncodeTimeSyncReply(&reply, payload, sizeof(payload)) ==
        VISION_V2_OK);
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_TIME_SYNC_REPLY,
        (uint16_t)block_sequence, t3_ms, payload, sizeof(payload), frame,
        &frame_length) == VISION_V2_OK);
    now_ms = t4_ms;
    InjectFrame(frame, frame_length, t4_ms, block_sequence, generation);
    return 0;
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
    uint8_t bad_frame[VISION_V2_MAX_FRAME];
    memcpy(bad_frame, frame, frame_length);
    bad_frame[frame_length - 1u] ^= 0x01u;
    InjectFrame(bad_frame, frame_length, now_ms, 4u, current_generation);
    VisionGetStatus(&status);
    TEST_CHECK(status.rx_count == accepted_rx && status.last_rx_ms == 0u &&
        status.crc_error_count > 0u);
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
    const uint32_t navigation_received_ms = freshness.received_time_ms;

    // 字段含NaN但CRC正确的导航帧也不能覆盖有效数据或更新时间。
    navigation_payload[1] = 0x00u;
    navigation_payload[2] = 0x00u;
    navigation_payload[3] = 0xc0u;
    navigation_payload[4] = 0x7fu;
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_NAVIGATION_COMMAND, 0u, 0u,
        navigation_payload, sizeof(navigation_payload), frame, &frame_length) ==
        VISION_V2_OK);
    const uint32_t published_before_invalid = stats.published;
    now_ms += 1u;
    InjectFrame(frame, frame_length, now_ms, 7u, current_generation);
    TEST_CHECK(MessageGetTopicStats(ROBOT_TOPIC_VISION_NAVIGATION, &stats) == MESSAGE_OK &&
        stats.published == published_before_invalid);
    TEST_CHECK(MessageReadSnapshot(navigation_subscriber, &navigation,
        sizeof(navigation), now_ms, &freshness) == MESSAGE_OK &&
        navigation.vx_m_s == 1.25f && freshness.received_time_ms == navigation_received_ms);
    VisionGetStatus(&status);
    TEST_CHECK(status.rx_count == accepted_rx + 1u &&
        status.last_rx_ms == navigation_received_ms);

    // 新连接先收到状态变化，再接纳新序号；首条导航恢复时旧瞄准也先失效。
    current_generation = 2u;
    usb_config.link_changed(usb_config.context, current_generation, USB_LINK_RESET);
    Vision_Status_t disconnected_status;
    VisionGetStatus(&disconnected_status);
    TEST_CHECK(disconnected_status.online == 0u && disconnected_status.has_valid_frame == 0u);
    ++now_ms;
    navigation_payload[0] = VISION_V2_NAV_TRANSLATION_VALID |
        VISION_V2_NAV_YAW_RATE_VALID;
    uint32_t vx_bits;
    const float recovered_vx_m_s = 0.75f;
    memcpy(&vx_bits, &recovered_vx_m_s, sizeof(vx_bits));
    for (size_t byte = 0u; byte < sizeof(vx_bits); ++byte)
        navigation_payload[1u + byte] = (uint8_t)(vx_bits >> (8u * byte));
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_NAVIGATION_COMMAND, 0x9000u, 1u,
        navigation_payload, sizeof(navigation_payload), frame, &frame_length) ==
        VISION_V2_OK);
    InjectFrame(frame, frame_length, now_ms, 7u, current_generation);
    TEST_CHECK(MessageReadSnapshot(navigation_subscriber, &navigation, sizeof(navigation),
        now_ms, &freshness) == MESSAGE_OK && navigation.valid_flags ==
        (VISION_V2_NAV_TRANSLATION_VALID | VISION_V2_NAV_YAW_RATE_VALID) &&
        navigation.vx_m_s == recovered_vx_m_s && freshness.received_time_ms == now_ms);
    TEST_CHECK(MessageReadSnapshot(aim_subscriber, &actual_aim, sizeof(actual_aim),
        now_ms, &freshness) == MESSAGE_OK && actual_aim.mode == 0u &&
        freshness.received_time_ms == now_ms);
    VisionGetStatus(&status);
    TEST_CHECK(status.online != 0u && status.rx_count == accepted_rx + 2u);

    // 任意序号的首条新瞄准恢复自己的分组，不改写本代导航数据。
    TEST_CHECK(VisionV2EncodeFrame(VISION_V2_AIM_COMMAND, 0xf123u, 1u,
        aim_payload, sizeof(aim_payload), frame, &frame_length) == VISION_V2_OK);
    ++now_ms;
    InjectFrame(frame, frame_length, now_ms, 8u, current_generation);
    TEST_CHECK(MessageReadSnapshot(aim_subscriber, &actual_aim, sizeof(actual_aim),
        now_ms, &freshness) == MESSAGE_OK && actual_aim.mode == aim.mode &&
        freshness.received_time_ms == now_ms);
    TEST_CHECK(MessageReadSnapshot(navigation_subscriber, &navigation,
        sizeof(navigation), now_ms, &freshness) == MESSAGE_OK &&
        navigation.vx_m_s == recovered_vx_m_s &&
        freshness.received_time_ms == now_ms - 1u);

    // 旧连接迟到块不更新任一话题。
    InjectFrame(frame, frame_length, now_ms, 8u, 1u);
    TEST_CHECK(MessageGetTopicStats(ROBOT_TOPIC_VISION_AIM, &stats) == MESSAGE_OK &&
        stats.published == 3u);
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

    Vision_Status_t status_before;
    Vision_Status_t status_after;
    VisionTimeSyncSnapshot_t sync = {0};
    VisionV2Frame_t sent_frame = {0};
    VisionV2TimeSyncProbe_t probe = {0};
    uint32_t mapped_local_ms = 0xdeadbeefu;
    const uint32_t reports_before = valid_command_reports;
    const uint32_t reloads_before = daemon_reloads;

    now_ms = 100u;
    VisionUpdateTx(1u, posture.quaternion_wxyz[0], posture.quaternion_wxyz[1],
        posture.quaternion_wxyz[2], posture.quaternion_wxyz[3], posture.yaw_rad,
        posture.yaw_rate_rad_s, posture.pitch_rad, posture.pitch_rate_rad_s, 30.0f, 2u);
    VisionGetStatus(&status_before);
    VisionSend();
    TEST_CHECK(tx_count == 1u && DecodeLastTx(&sent_frame) == 0);
    TEST_CHECK(sent_frame.type == VISION_V2_TIME_SYNC_PROBE &&
        VisionV2DecodeTimeSyncProbe(sent_frame.payload, sent_frame.payload_length,
            &probe) == VISION_V2_OK && probe.request_id == 1u &&
        sent_frame.source_time_ms == 100u);

    // 请求尚未完成时不重复探测；普通状态报告仍按原节奏发送。
    now_ms = 150u;
    VisionSend();
    TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
        sent_frame.type == VISION_V2_POSTURE_STATUS);
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.request_pending != 0u && sync.request_id == probe.request_id);

    // 时钟样本仅使用完整帧末RxEvent时间；对时帧不能刷新业务在线状态。
    TEST_CHECK(InjectTimeReply(probe.request_id, 120u, 180u, 200u, 9u,
        current_generation) == 0);
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 1u && sync.synchronized == 0u &&
        sync.probes_accepted == 1u && sync.last_round_trip_ms == 40u);
    VisionGetStatus(&status_after);
    TEST_CHECK(status_after.rx_count == status_before.rx_count &&
        status_after.last_rx_ms == status_before.last_rx_ms &&
        valid_command_reports == reports_before && daemon_reloads == reloads_before);

    // 三个500ms周期的低RTT样本建立映射，偏差定义为PC时钟减MCU时钟。
    for (uint32_t sample = 1u; sample < TIME_MAPPING_REQUIRED_SAMPLES; ++sample)
    {
        const uint32_t t1_ms = 100u + sample * 500u;
        now_ms = t1_ms;
        VisionSend();
        TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_TIME_SYNC_PROBE);
        TEST_CHECK(VisionV2DecodeTimeSyncProbe(sent_frame.payload,
            sent_frame.payload_length, &probe) == VISION_V2_OK &&
            sent_frame.source_time_ms == t1_ms);
        TEST_CHECK(InjectTimeReply(probe.request_id, t1_ms + 5u,
            t1_ms + 6u, t1_ms + 10u, 9u + sample,
            current_generation) == 0);
        VisionSend();
    }
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 3u && sync.synchronized != 0u &&
        sync.remote_minus_local_ms == 1 && sync.last_round_trip_ms == 9u);
    TEST_CHECK(VisionMapRemoteTime(now_ms - 4u, now_ms, &mapped_local_ms) ==
        VISION_TIME_SYNC_OK && mapped_local_ms == now_ms - 5u);

    // 错编号回应不覆盖映射；未答请求超过100ms后计为超时。
    now_ms = 1600u;
    VisionSend();
    TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
        sent_frame.type == VISION_V2_TIME_SYNC_PROBE);
    TEST_CHECK(VisionV2DecodeTimeSyncProbe(sent_frame.payload,
        sent_frame.payload_length, &probe) == VISION_V2_OK);
    TEST_CHECK(InjectTimeReply(probe.request_id + 1u, 1602u, 1603u, 1604u,
        15u, current_generation) == 0);
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.rejected_replies == 1u && sync.sample_count == 3u &&
        sync.synchronized != 0u);
    now_ms = 1701u;
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.probe_timeouts == 1u && sync.request_pending == 0u);

    // 两秒没有新样本后映射失效；USB代号变化立即撤销旧偏差并拒绝旧代回应。
    now_ms = 3111u;
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.synchronized == 0u && sync.sample_age_ms == 2001u);
    TEST_CHECK(VisionMapRemoteTime(now_ms, now_ms, &mapped_local_ms) ==
        VISION_TIME_SYNC_ERR_EXPIRED && mapped_local_ms == 1105u);
    current_generation = 3u;
    if (usb_config.link_changed != NULL)
        usb_config.link_changed(usb_config.context, current_generation, USB_LINK_RESET);
    now_ms += 1u;
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 0u && sync.synchronized == 0u &&
        sync.transport_generation == current_generation);
    TEST_CHECK(tx_count >= 8u);

    // 新连接只接受本代响应；随后连续三份样本重新建立映射。
    TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
        sent_frame.type == VISION_V2_TIME_SYNC_PROBE);
    TEST_CHECK(VisionV2DecodeTimeSyncProbe(sent_frame.payload,
        sent_frame.payload_length, &probe) == VISION_V2_OK);
    const uint32_t reconnect_t1_ms = sent_frame.source_time_ms;
    TEST_CHECK(InjectTimeReply(probe.request_id, reconnect_t1_ms + 5u,
        reconnect_t1_ms + 6u, reconnect_t1_ms + 10u, 16u,
        current_generation - 1u) == 0);
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 0u && sync.request_pending != 0u);
    TEST_CHECK(InjectTimeReply(probe.request_id, reconnect_t1_ms + 5u,
        reconnect_t1_ms + 6u, reconnect_t1_ms + 10u, 17u,
        current_generation) == 0);
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 1u && sync.synchronized == 0u);
    for (uint32_t sample = 1u; sample < TIME_MAPPING_REQUIRED_SAMPLES; ++sample)
    {
        const uint32_t t1_ms = reconnect_t1_ms + sample * 500u;
        now_ms = t1_ms;
        VisionSend();
        TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_TIME_SYNC_PROBE);
        TEST_CHECK(VisionV2DecodeTimeSyncProbe(sent_frame.payload,
            sent_frame.payload_length, &probe) == VISION_V2_OK);
        TEST_CHECK(InjectTimeReply(probe.request_id, t1_ms + 5u,
            t1_ms + 6u, t1_ms + 10u, 17u + sample,
            current_generation) == 0);
        VisionSend();
    }
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.sample_count == 3u && sync.synchronized != 0u &&
        sync.transport_generation == current_generation);

    // 入队失败不保留在途请求；错误被报告，下一500ms周期可以重新提交。
    const uint32_t failed_probe_time = reconnect_t1_ms + 1500u;
    now_ms = failed_probe_time;
    next_submit_status = USB_BUSY;
    VisionSend();
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.request_pending == 0u &&
        sync.last_result == VISION_TIME_SYNC_RESULT_SEND_REJECTED);
    now_ms = failed_probe_time + 500u;
    VisionSend();
    TEST_CHECK(DecodeLastTx(&sent_frame) == 0 &&
        sent_frame.type == VISION_V2_TIME_SYNC_PROBE);
    TEST_CHECK(VisionV2DecodeTimeSyncProbe(sent_frame.payload,
        sent_frame.payload_length, &probe) == VISION_V2_OK);
    TEST_CHECK(VisionGetTimeSyncSnapshot(&sync) == VISION_TIME_SYNC_OK &&
        sync.request_pending != 0u && sync.request_id == probe.request_id);

    // 对时探测不替代原有的双向状态报告，周期发送顺序保持不变。
    now_ms += 50u;
    VisionSend();
    TEST_CHECK(DecodeLastTx(&sent_frame) == 0);
    if (sent_frame.type == VISION_V2_TIME_SYNC_PROBE)
    {
        now_ms += 50u;
        VisionSend();
        TEST_CHECK(DecodeLastTx(&sent_frame) == 0);
    }
    TEST_CHECK(sent_frame.type == VISION_V2_POSTURE_STATUS ||
        sent_frame.type == VISION_V2_REFEREE_SUMMARY);

    TEST_CHECK(valid_command_reports == reports_before &&
        daemon_reloads == reloads_before);

    // V2轮询应把INS样本序号和源时刻原样送入独立IMU状态帧。
    VisionIMUSample_t sample = {0};
    VisionV2IMUSampleStatus_t decoded_sample = {0};
    uint8_t found_report = 0u;
    uint32_t expected_sequence = 0u;
    uint32_t expected_source_time = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += 50u;
        sample = (VisionIMUSample_t){
            .valid = 1u,
            .sample_sequence = 0x100u + attempt,
            .sample_time_ms = now_ms,
            .acceleration_m_s2 = {1.0f, -0.5f, 9.81f},
        };
        expected_sequence = sample.sample_sequence;
        expected_source_time = sample.sample_time_ms;
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == expected_source_time &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 1u &&
        decoded_sample.sample_sequence == expected_sequence &&
        decoded_sample.acceleration_m_s2[0] == 1.0f &&
        decoded_sample.acceleration_m_s2[1] == -0.5f &&
        decoded_sample.acceleration_m_s2[2] == 9.81f);

    // 失效标记等待本类发送槽时仍保留，出线后时间、序号和三轴均清零。
    sample = (VisionIMUSample_t){0};
    TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += 50u;
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == 0u &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 0u &&
        decoded_sample.sample_sequence == 0u &&
        decoded_sample.acceleration_m_s2[0] == 0.0f &&
        decoded_sample.acceleration_m_s2[1] == 0.0f &&
        decoded_sample.acceleration_m_s2[2] == 0.0f);

    // 传感器停止更新后，消息中心仍保留诊断记录，但发送端不能续发旧有效值。
    sample = (VisionIMUSample_t){
        .valid = 1u, .sample_sequence = 0x200u, .sample_time_ms = now_ms,
        .acceleration_m_s2 = {0.0f, 0.0f, 9.81f},
    };
    TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 6u; ++attempt)
    {
        now_ms += 50u;
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == 0u &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 0u);

    // 毫秒时钟和BMI088序号都允许回绕；无符号年龄在时钟回绕后仍按5ms边界判断。
    now_ms = UINT32_MAX - 2u;
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += attempt == 0u ? 0u : 50u;
        sample = (VisionIMUSample_t){
            .valid = 1u,
            .sample_sequence = UINT32_MAX,
            .sample_time_ms = now_ms - 5u,
            .acceleration_m_s2 = {0.25f, -0.5f, 9.75f},
        };
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == sample.sample_time_ms &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 1u &&
        decoded_sample.sample_sequence == UINT32_MAX);

    // BMI088序号绕过保留的0后从1继续，线路帧应逐样本保留这个变化。
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += 50u;
        sample.sample_sequence = 1u;
        sample.sample_time_ms = now_ms;
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == sample.sample_time_ms &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 1u &&
        decoded_sample.sample_sequence == 1u);

    // 超过边界1ms时发布无效状态；每次重发样本只用于保持快照可读，不刷新其源时刻。
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += 50u;
        sample.sample_time_ms = now_ms - 6u;
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == 0u &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 0u);

    // 消息中心快照本身过期后，也要发送撤销状态，不能让上位机永久保留最后有效值。
    found_report = 0u;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
        now_ms += 50u;
        sample.sample_time_ms = now_ms - 301u;
        TEST_CHECK(VisionUpdateIMUSample(&sample) == VISION_IMU_SAMPLE_UPDATE_OK);
        const uint32_t previous_tx_count = tx_count;
        VisionSend();
        if (tx_count != previous_tx_count && DecodeLastTx(&sent_frame) == 0 &&
            sent_frame.type == VISION_V2_IMU_SAMPLE_STATUS)
        {
            found_report = 1u;
            break;
        }
    }
    TEST_CHECK(found_report != 0u && sent_frame.source_time_ms == 0u &&
        VisionV2DecodeIMUSampleStatus(sent_frame.payload, sent_frame.payload_length,
            &decoded_sample) == VISION_V2_OK && decoded_sample.valid == 0u);
    return 0;
}

int main(void)
{
    TEST_CHECK(RobotTopicsInit() != 0u);
    VisionIMUSample_t sample_before_init = {
        .valid = 1u, .sample_sequence = 1u, .sample_time_ms = 0u,
        .acceleration_m_s2 = {0.0f, 0.0f, 9.81f},
    };
    TEST_CHECK(VisionUpdateIMUSample(&sample_before_init) ==
        VISION_IMU_SAMPLE_UPDATE_ERR_NOT_INITIALIZED);
    TEST_CHECK(VisionInit(NULL) != NULL);
    TEST_CHECK(usb_config.receive != NULL && daemon_config.transport_timeout_ms == 300u);
    now_ms = 1u;
    VisionIMUSample_t zero_time_sample = {
        .valid = 1u, .sample_sequence = 1u, .sample_time_ms = 0u,
        .acceleration_m_s2 = {0.0f, 0.0f, 9.81f},
    };
    TEST_CHECK(VisionUpdateIMUSample(&zero_time_sample) == VISION_IMU_SAMPLE_UPDATE_OK);
    TEST_CHECK(VisionUpdateIMUSample(NULL) == VISION_IMU_SAMPLE_UPDATE_ERR_ARGUMENT);
    zero_time_sample.sample_sequence = 0u;
    TEST_CHECK(VisionUpdateIMUSample(&zero_time_sample) == VISION_IMU_SAMPLE_UPDATE_ERR_VALUE);
    zero_time_sample.sample_sequence = 1u;
    zero_time_sample.acceleration_m_s2[0] = NAN;
    TEST_CHECK(VisionUpdateIMUSample(&zero_time_sample) == VISION_IMU_SAMPLE_UPDATE_ERR_VALUE);
    zero_time_sample.acceleration_m_s2[0] = INFINITY;
    TEST_CHECK(VisionUpdateIMUSample(&zero_time_sample) == VISION_IMU_SAMPLE_UPDATE_ERR_VALUE);
    TEST_CHECK(CheckInboundTopics() == 0);
    TEST_CHECK(CheckOutboundReports() == 0);
    return 0;
}
