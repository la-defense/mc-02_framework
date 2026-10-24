/**
 * @file master_process.c
 * @brief 视觉数据收发, SP+CRC16 协议(与上位机 sp_vision_25 对齐)
 *        按构建配置使用SP兼容帧或V2业务帧，并支持USB VCP或UART传输。
 */
#include "master_process.h"
#include "crc.h"
#include "vision_protocol_v2.h"
#include "message_freshness.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#if defined(MC02_VISION_PROTOCOL_V2)
#include "topics/robot_topics.h"
#include "message_center.h"
#include "referee_snapshot.h"
#include "rm_referee.h"
#endif

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint32_t vision_rx_count = 0;
static uint32_t vision_tx_count = 0;
static uint32_t vision_tx_drop_count = 0;
static uint32_t vision_crc_error_count = 0;
static uint32_t vision_last_rx_ms = 0;
static uint8_t vision_has_valid_frame;
#if defined(MC02_VISION_PROTOCOL_V2)
static VisionV2Stream_t vision_v2_stream;
#else
static VisionSPStream_t vision_stream;
#endif
static volatile uint8_t vision_stream_reset_requested;
#if defined(MC02_VISION_PROTOCOL_V2)
static MessagePublisher_t *vision_v2_aim_publisher;
static MessagePublisher_t *vision_v2_navigation_publisher;
static MessagePublisher_t *vision_v2_posture_publisher;
static MessageSubscriber_t *vision_v2_posture_subscriber;
static uint16_t vision_v2_last_sequence[2];
static uint8_t vision_v2_sequence_valid[2];
static uint32_t vision_v2_sequence_generation;
static uint8_t vision_v2_sequence_generation_valid;
static uint32_t vision_v2_last_tx_ms;
static uint8_t vision_v2_has_tx;
static uint8_t vision_v2_next_tx_type;
static uint16_t vision_v2_tx_sequence;
#endif

#ifdef VISION_USE_UART
#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;
#endif

#ifdef VISION_USE_VCP
#include "usb_transport.h"
static uint32_t vision_usb_generation;
#endif

#if !defined(MC02_VISION_PROTOCOL_V2)
// 线上CRC通过后继续检查实际字段；解析不授予使能，控制端仍检查范围和当前许可。
static void VisionFrameReceived(void *context, const Vision_Recv_s *packet,
                                uint32_t received_ms, uint32_t generation)
{
    (void)context;
    if (packet->mode > 2u || !isfinite(packet->yaw) || !isfinite(packet->yaw_vel) ||
        !isfinite(packet->yaw_acc) || !isfinite(packet->pitch) || !isfinite(packet->pitch_vel) ||
        !isfinite(packet->pitch_acc))
        return;
    taskENTER_CRITICAL();
    uint8_t current = (uint8_t)(vision_stream_reset_requested == 0u &&
        (uint32_t)(HAL_GetTick() - received_ms) <= MESSAGE_DEFAULT_MAX_AGE_MS);
#ifdef VISION_USE_VCP
    current = (uint8_t)(current && USBTransportGenerationIsCurrent(generation));
#else
    (void)generation;
#endif
    if (current)
    {
        recv_data = *packet;
        vision_rx_count++;
        vision_last_rx_ms = received_ms;
        vision_has_valid_frame = 1u;
#ifdef VISION_USE_VCP
        vision_usb_generation = generation;
        USBTransportReportValidCommand(generation, received_ms);
#endif
        DaemonReload(vision_daemon_instance);
    }
    taskEXIT_CRITICAL();
}
#endif

#if defined(MC02_VISION_PROTOCOL_V2)
// 调用者持有临界区，避免离线回调同时清除序号状态。
static uint8_t VisionV2SequenceIsNew(uint8_t type_index, uint16_t sequence)
{
    if (vision_v2_sequence_valid[type_index] == 0u)
        return 1u;
    const uint16_t distance = (uint16_t)(sequence - vision_v2_last_sequence[type_index]);
    return (uint8_t)(distance != 0u && distance <= 0x7fffu);
}

/* 新USB代际首个合法帧到来时清掉另一业务类型的旧快照，避免短暂重用上次连接命令。 */
static uint8_t VisionV2InvalidateInputSnapshots(const VisionV2Frame_t *frame,
                                                uint32_t now_ms)
{
    const VisionV2AimCommand_t no_aim = {0};
    const VisionV2NavigationCommand_t no_navigation = {0};
    return (uint8_t)(MessagePublishAt(vision_v2_aim_publisher, &no_aim,
        frame->source_time_ms, frame->received_time_ms, now_ms,
        MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK &&
        MessagePublishAt(vision_v2_navigation_publisher, &no_navigation,
        frame->source_time_ms, frame->received_time_ms, now_ms,
        MESSAGE_CLOCK_REMOTE_RAW) == MESSAGE_OK);
}

// 每类命令独立检查序号与本地接收年龄，再发布原始远端时刻。
static void VisionV2FrameReceived(void *context, const VisionV2Frame_t *frame)
{
    (void)context;
    if (frame == NULL)
        return;
#ifdef VISION_USE_VCP
    // USB重连后旧块可能仍在任务队列里，不能让它恢复旧连接的命令。
    if (!USBTransportGenerationIsCurrent(frame->generation))
        return;
#endif
    const uint32_t now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - frame->received_time_ms) > VISION_V2_NAV_MAX_AGE_MS)
        return;

    MessageStatus_t message_status = MESSAGE_ERR_TOPIC;
    uint8_t type_index = 0u;
    VisionV2AimCommand_t aim;
    VisionV2NavigationCommand_t navigation;
    if (frame->type == VISION_V2_AIM_COMMAND)
    {
        if ((uint32_t)(now_ms - frame->received_time_ms) > VISION_V2_AIM_MAX_AGE_MS ||
            VisionV2DecodeAim(frame->payload, frame->payload_length, &aim) != VISION_V2_OK)
            return;
    }
    else if (frame->type == VISION_V2_NAVIGATION_COMMAND)
    {
        type_index = 1u;
        if (VisionV2DecodeNavigation(frame->payload, frame->payload_length,
                                     &navigation) != VISION_V2_OK)
            return;
    }
    else
        return;

    taskENTER_CRITICAL();
    if (vision_v2_sequence_generation_valid == 0u ||
        frame->generation != vision_v2_sequence_generation)
    {
        if (vision_v2_sequence_generation_valid != 0u &&
            VisionV2InvalidateInputSnapshots(frame, now_ms) == 0u)
        {
            taskEXIT_CRITICAL();
            return;
        }
        memset(vision_v2_sequence_valid, 0, sizeof(vision_v2_sequence_valid));
        vision_v2_sequence_generation = frame->generation;
        vision_v2_sequence_generation_valid = 1u;
    }
    // 首帧或整条链路已过期后，允许上位机从任意新序号恢复。
    const uint8_t link_restart = (uint8_t)(vision_has_valid_frame == 0u ||
        (uint32_t)(now_ms - vision_last_rx_ms) > 300u);
    if (link_restart != 0u)
        memset(vision_v2_sequence_valid, 0, sizeof(vision_v2_sequence_valid));

    if (frame->type == VISION_V2_AIM_COMMAND)
    {
        if (!VisionV2SequenceIsNew(type_index, frame->sequence))
        {
            taskEXIT_CRITICAL();
            return;
        }
        message_status = MessagePublishAt(vision_v2_aim_publisher, &aim,
            frame->source_time_ms, frame->received_time_ms, now_ms, MESSAGE_CLOCK_REMOTE_RAW);
    }
    else
    {
        if (!VisionV2SequenceIsNew(type_index, frame->sequence))
        {
            taskEXIT_CRITICAL();
            return;
        }
        message_status = MessagePublishAt(vision_v2_navigation_publisher, &navigation,
            frame->source_time_ms, frame->received_time_ms, now_ms, MESSAGE_CLOCK_REMOTE_RAW);
    }

    if (message_status != MESSAGE_OK)
    {
        taskEXIT_CRITICAL();
        return;
    }
    vision_v2_last_sequence[type_index] = frame->sequence;
    vision_v2_sequence_valid[type_index] = 1u;
    vision_rx_count++;
    vision_last_rx_ms = frame->received_time_ms;
    vision_has_valid_frame = 1u;
#ifdef VISION_USE_VCP
    vision_usb_generation = frame->generation;
#endif
    DaemonReload(vision_daemon_instance);
    taskEXIT_CRITICAL();
#ifdef VISION_USE_VCP
    USBTransportReportValidCommand(frame->generation, frame->received_time_ms);
#endif
}
#endif

// 只有解析调用者清理候选数组；其他任务只置请求标志，避免清零与解析交错。
static void VisionFeedBlock(const uint8_t *data, size_t length, uint32_t received_ms,
                            uint32_t sequence, uint32_t generation)
{
    taskENTER_CRITICAL();
    uint8_t reset = vision_stream_reset_requested;
    vision_stream_reset_requested = 0u;
    taskEXIT_CRITICAL();
    if (reset)
    {
#if !defined(MC02_VISION_PROTOCOL_V2)
        VisionProtocolStreamReset(&vision_stream);
#else
        VisionV2StreamReset(&vision_v2_stream);
#endif
    }
    uint32_t errors = 0u;
#if defined(MC02_VISION_PROTOCOL_V2)
    (void)VisionV2StreamFeed(&vision_v2_stream, data, length, received_ms, sequence,
        generation, HAL_GetTick(), VisionV2FrameReceived, NULL, &errors);
#else
    (void)VisionProtocolStreamFeed(&vision_stream, data, length, received_ms, sequence,
        generation, HAL_GetTick(), VisionFrameReceived, NULL, &errors);
#endif
    taskENTER_CRITICAL();
    vision_crc_error_count += errors;
    taskEXIT_CRITICAL();
}

#ifdef VISION_USE_UART
static void VisionUartRxCallback(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    if (block == NULL) return;
    VisionFeedBlock(block->data, block->length, block->received_ms, block->sequence, 1u);
}
#endif

// 离线时清旧命令、接收时间和有效标记；UART 配置还会重新准备接收。
static void VisionOfflineCallback(void *id)
{
    (void)id;
    taskENTER_CRITICAL();
    memset(&recv_data, 0, sizeof(recv_data));
    vision_last_rx_ms = 0u;
    vision_has_valid_frame = 0u;
    vision_stream_reset_requested = 1u;
#if defined(MC02_VISION_PROTOCOL_V2)
    memset(vision_v2_sequence_valid, 0, sizeof(vision_v2_sequence_valid));
#endif
    taskEXIT_CRITICAL();
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    BSP_LOG_EVENT_0(EV_MODULES_MASTER_MACHINE_MASTER_PROCESS_C_01);
}

void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count)
{
    taskENTER_CRITICAL();
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    send_data.mode = mode;

    send_data.q[0] = q0;
    send_data.q[1] = q1;
    send_data.q[2] = q2;
    send_data.q[3] = q3;

    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
    taskEXIT_CRITICAL();

#if defined(MC02_VISION_PROTOCOL_V2)
    VisionV2PostureStatus_t posture = {
        .valid_flags = mode != 0u ?
            (VISION_V2_POSTURE_ORIENTATION_VALID | VISION_V2_POSTURE_RATE_VALID) : 0u,
        .robot_state = mode != 0u ? 1u : 0u,
        .quaternion_wxyz = {q0, q1, q2, q3},
        .yaw_rad = yaw,
        .yaw_rate_rad_s = yaw_vel,
        .pitch_rad = pitch,
        .pitch_rate_rad_s = pitch_vel,
    };
    if (!isfinite(q0) || !isfinite(q1) || !isfinite(q2) || !isfinite(q3) ||
        !isfinite(yaw) || !isfinite(yaw_vel) || !isfinite(pitch) || !isfinite(pitch_vel))
    {
        memset(&posture, 0, sizeof(posture));
    }
    (void)MessagePublishNow(vision_v2_posture_publisher, &posture);
#endif
}

#ifdef VISION_USE_VCP
static void VisionVcpRxCallback(void *context, const USBRxBlock_t *block)
{
    (void)context;
    if (block == NULL) return;
    VisionFeedBlock(block->data, block->length, block->received_ms, block->sequence, block->generation);
}
static void VisionVcpLinkChanged(void *context, uint32_t generation, USBLinkReason_t reason)
{
    (void)context; (void)generation; (void)reason;
    taskENTER_CRITICAL();
    // 新连接可能已先收到首帧；迟一点执行的连接通知不能再清掉这份新数据。
    if (!vision_has_valid_frame || vision_usb_generation != generation ||
        !USBTransportGenerationIsCurrent(generation))
    {
        memset(&recv_data, 0, sizeof(recv_data));
        vision_has_valid_frame = 0u;
        vision_last_rx_ms = 0u;
        vision_stream_reset_requested = 1u;
    }
    taskEXIT_CRITICAL();
}
static void VisionVcpTxCompleted(void *context, const USBTxEvent_t *event)
{
    (void)context;
    if (event->result != USB_TX_COMPLETE)
    { taskENTER_CRITICAL(); ++vision_tx_drop_count; taskEXIT_CRITICAL(); }
}
#endif

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));
    vision_last_rx_ms = 0u;
    vision_has_valid_frame = 0u;
#if !defined(MC02_VISION_PROTOCOL_V2)
    VisionProtocolStreamReset(&vision_stream);
#else
    VisionV2StreamReset(&vision_v2_stream);
#endif
    vision_stream_reset_requested = 0u;
#if defined(MC02_VISION_PROTOCOL_V2)
    memset(vision_v2_sequence_valid, 0, sizeof(vision_v2_sequence_valid));
    vision_v2_sequence_generation = 0u;
    vision_v2_sequence_generation_valid = 0u;
    vision_v2_has_tx = 0u;
    vision_v2_next_tx_type = 0u;
    vision_v2_tx_sequence = 0u;
    if (RobotTopicsInit() == 0u)
        return NULL;
    vision_v2_aim_publisher = MessagePublisherRegister(ROBOT_TOPIC_VISION_AIM,
        ROBOT_TOPIC_ENDPOINT_VISION);
    vision_v2_navigation_publisher = MessagePublisherRegister(ROBOT_TOPIC_VISION_NAVIGATION,
        ROBOT_TOPIC_ENDPOINT_VISION);
    vision_v2_posture_publisher = MessagePublisherRegister(ROBOT_TOPIC_VISION_POSTURE,
        ROBOT_TOPIC_ENDPOINT_INS);
    vision_v2_posture_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_VISION_POSTURE,
        ROBOT_TOPIC_ENDPOINT_VISION);
    if (vision_v2_aim_publisher == NULL || vision_v2_navigation_publisher == NULL ||
        vision_v2_posture_publisher == NULL || vision_v2_posture_subscriber == NULL)
        return NULL;
#endif

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count =
#if defined(MC02_VISION_PROTOCOL_V2)
            30, // V2链路整体在线期限为300ms，瞄准与导航另行按各自期限失效。
#else
            5, // SP旧协议在线期限为50ms。
#endif
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    if (vision_daemon_instance == NULL)
        return NULL;

#ifdef VISION_USE_VCP
    (void)_handle;
    USBTransportConfig_t conf = {VisionVcpRxCallback, VisionVcpTxCompleted, VisionVcpLinkChanged, NULL};
    if (USBTransportInit(&conf) != USB_OK)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
#elif defined(VISION_USE_UART)
    if (_handle == NULL)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
    USART_Init_Config_s conf = {0};
    conf.context = &vision_stream;
    conf.module_callback = VisionUartRxCallback;
    conf.recv_buff_size = VISION_RECV_SIZE * 2;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);
    if (vision_usart_instance == NULL)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
#else
    (void)_handle;
    BSP_LOG_EVENT_0(EV_MODULES_MASTER_MACHINE_MASTER_PROCESS_C_02);
    (void)DaemonUnregister(vision_daemon_instance);
    vision_daemon_instance = NULL;
    return NULL;
#endif

    return &recv_data;
}

// 即使没有离线回调，读取或发送时也检查 50 ms 时限，避免旧目标继续使用。
static void VisionExpireIfStale(uint32_t now_ms)
{
    taskENTER_CRITICAL();
#if defined(MC02_VISION_PROTOCOL_V2)
    const uint32_t link_max_age_ms = 300u;
#else
    const uint32_t link_max_age_ms = MESSAGE_DEFAULT_MAX_AGE_MS;
#endif
    uint8_t expired = (uint8_t)((uint32_t)(now_ms - vision_last_rx_ms) > link_max_age_ms);
#ifdef VISION_USE_VCP
    expired = (uint8_t)(expired || !USBTransportGenerationIsCurrent(vision_usb_generation));
#endif
    if (vision_has_valid_frame != 0u && expired)
    {
        memset(&recv_data, 0, sizeof(recv_data));
        vision_has_valid_frame = 0u;
    }
    taskEXIT_CRITICAL();
}

#ifdef VISION_USE_UART
static USARTTxEvent_t vision_uart_last_tx;
static void VisionTxCompleted(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    taskENTER_CRITICAL();vision_uart_last_tx=*event;
    if(event->result!=USART_TX_COMPLETE)++vision_tx_drop_count;
    taskEXIT_CRITICAL();
}
#endif

#if defined(MC02_VISION_PROTOCOL_V2)
static uint8_t VisionV2BuildRefereeSummary(VisionV2RefereeSummary_t *summary)
{
    if (summary == NULL)
        return 0u;
    RefereeSnapshot_t snapshot;
    if (RefereeGetSnapshot(&snapshot) != REFEREE_OK)
        return 0u;

    memset(summary, 0, sizeof(*summary));
    const RefereeCommandSnapshot_t *robot = &snapshot.command[REF_CMD_ROBOT_STATE];
    if (robot->state == REFEREE_DATA_FRESH && isfinite(robot->data.robot_state.bullet_speed_limit_mps))
    {
        summary->valid_flags |= VISION_V2_REF_ROBOT_STATE_VALID;
        summary->robot_id = robot->data.robot_state.robot_id;
        summary->current_hp = robot->data.robot_state.current_hp;
        summary->maximum_hp = robot->data.robot_state.maximum_hp;
        summary->chassis_power_limit_w = robot->data.robot_state.chassis_power_limit_w;
        summary->bullet_speed_limit_m_s = robot->data.robot_state.bullet_speed_limit_mps;
        summary->power_output_flags = robot->data.robot_state.power_output_bits;
        summary->robot_state_source_ms = robot->source_rx_ms;
    }

    const RefereeCommandSnapshot_t *power = &snapshot.command[REF_CMD_POWER_HEAT];
    if (power->state == REFEREE_DATA_FRESH)
    {
        summary->valid_flags |= VISION_V2_REF_POWER_HEAT_VALID;
        summary->buffer_energy_j = power->data.power_heat.buffer_energy_j;
        summary->heat_17mm = power->data.power_heat.heat_17mm;
        summary->heat_42mm = power->data.power_heat.heat_42mm;
        summary->power_heat_source_ms = power->source_rx_ms;
    }

    const RefereeCommandSnapshot_t *shot = &snapshot.command[REF_CMD_SHOOT_DATA];
    if ((shot->state == REFEREE_DATA_FRESH || shot->state == REFEREE_DATA_EVENT_RECORDED) &&
        isfinite(shot->data.shoot_data.initial_speed_mps))
    {
        summary->valid_flags |= VISION_V2_REF_SHOOT_RECORD_VALID;
        summary->last_shot_frequency_hz = shot->data.shoot_data.frequency_hz;
        summary->last_shot_speed_m_s = shot->data.shoot_data.initial_speed_mps;
        summary->shoot_record_source_ms = shot->source_rx_ms;
    }
    return 1u;
}

static void VisionV2NoteTxResult(uint8_t accepted)
{
    taskENTER_CRITICAL();
    if (accepted != 0u)
    {
        vision_tx_count++;
        vision_v2_tx_sequence++;
    }
    else
        vision_tx_drop_count++;
    taskEXIT_CRITICAL();
}

static void VisionV2SubmitFrame(uint8_t type, const uint8_t *payload, size_t payload_length)
{
    uint8_t frame[VISION_V2_MAX_FRAME];
    size_t frame_length = 0u;
    const uint16_t sequence = vision_v2_tx_sequence;
    if (VisionV2EncodeFrame(type, sequence, HAL_GetTick(), payload, payload_length,
                            frame, &frame_length) != VISION_V2_OK)
    {
        VisionV2NoteTxResult(0u);
        return;
    }

#ifdef VISION_USE_VCP
    uint32_t id = 0u;
    VisionV2NoteTxResult((uint8_t)(USBTransportSubmit(frame, frame_length, &id) == USB_ACCEPTED));
#elif defined(VISION_USE_UART)
    const USARTTxRequest_t request = {
        .data = frame,
        .length = frame_length,
        .mode = USART_TRANSFER_DMA,
        .timeout_ms = 100u,
        .complete = VisionTxCompleted,
        .guard = NULL,
        .context = NULL,
    };
    uint32_t id = 0u;
    VisionV2NoteTxResult((uint8_t)(USARTSubmitTx(vision_usart_instance, &request, &id) ==
                                   USART_SEND_ACCEPTED));
#else
    (void)frame;
    (void)frame_length;
    VisionV2NoteTxResult(0u);
#endif
}

static void VisionSendV2(void)
{
    const uint32_t now_ms = HAL_GetTick();
    VisionExpireIfStale(now_ms);
    if (vision_v2_has_tx != 0u && (uint32_t)(now_ms - vision_v2_last_tx_ms) < 50u)
        return;
    vision_v2_has_tx = 1u;
    vision_v2_last_tx_ms = now_ms;

    uint8_t payload[VISION_V2_MAX_PAYLOAD];
    size_t payload_length = 0u;
    if (vision_v2_next_tx_type == 0u)
    {
        VisionV2PostureStatus_t posture;
        MessageFreshness_t freshness;
        if (MessageReadSnapshot(vision_v2_posture_subscriber, &posture, sizeof(posture),
                                now_ms, &freshness) != MESSAGE_OK ||
            VisionV2EncodePosture(&posture, payload, sizeof(payload)) != VISION_V2_OK)
        {
            VisionV2NoteTxResult(0u);
            vision_v2_next_tx_type = 1u;
            return;
        }
        (void)VisionV2GetPayloadSize(VISION_V2_POSTURE_STATUS, &payload_length);
        VisionV2SubmitFrame(VISION_V2_POSTURE_STATUS, payload, payload_length);
    }
    else
    {
        VisionV2RefereeSummary_t summary;
        if (VisionV2BuildRefereeSummary(&summary) == 0u ||
            VisionV2EncodeRefereeSummary(&summary, payload, sizeof(payload)) != VISION_V2_OK)
        {
            VisionV2NoteTxResult(0u);
            vision_v2_next_tx_type = 0u;
            return;
        }
        (void)VisionV2GetPayloadSize(VISION_V2_REFEREE_SUMMARY, &payload_length);
        VisionV2SubmitFrame(VISION_V2_REFEREE_SUMMARY, payload, payload_length);
    }
    vision_v2_next_tx_type ^= 1u;
}
#endif

void VisionSend(void)
{
#if defined(MC02_VISION_PROTOCOL_V2)
    VisionSendV2();
    return;
#else
    VisionExpireIfStale(HAL_GetTick());
    // 两种传输都在提交时复制；组包使用本次独占的局部内容。
    Vision_Send_s tx_pkt;
    taskENTER_CRITICAL();
    tx_pkt = send_data;
    taskEXIT_CRITICAL();
    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    uint32_t crc;
    // CRC 失败时不提交本帧，旧缓冲区不能冒充本次有效状态。
    if (CRCCompute(CRC16_SP_REFEREE, (const uint8_t *)&tx_pkt,
                   sizeof(Vision_Send_s) - 2u, &crc) != CRC_OK)
        return;
    tx_pkt.crc16 = (uint16_t)crc;

#ifdef VISION_USE_VCP
    uint32_t id;
    USBStatus_t usb_status = USBTransportSubmit((const uint8_t *)&tx_pkt, sizeof(tx_pkt), &id);
    taskENTER_CRITICAL();
    if (usb_status == USB_ACCEPTED)
        vision_tx_count++;
    else
        vision_tx_drop_count++;
    taskEXIT_CRITICAL();
#elif defined(VISION_USE_UART)
    uint32_t id;
    USARTTxRequest_t request={(const uint8_t *)&tx_pkt,sizeof(tx_pkt),USART_TRANSFER_DMA,
        20u,VisionTxCompleted,NULL,NULL};
    if(USARTSubmitTx(vision_usart_instance,&request,&id)==USART_SEND_ACCEPTED)
        vision_tx_count++;
    else { taskENTER_CRITICAL();++vision_tx_drop_count;taskEXIT_CRITICAL(); }
#endif
#endif
}

void VisionGetStatus(Vision_Status_t *status)
{
    VisionGetSnapshot(NULL, status);
}

void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status)
{
    VisionExpireIfStale(HAL_GetTick());
    taskENTER_CRITICAL();
    // 两个输出可各自省略；需要哪份数据就复制哪份，整个复制过程放在一起。
    if (frame != NULL)
        *frame = recv_data;
    if (status != NULL)
    {
        status->has_valid_frame = vision_has_valid_frame;
        status->online = (vision_has_valid_frame != 0u && vision_daemon_instance != NULL &&
                          DaemonIsOnline(vision_daemon_instance) > 0) ? 1 : 0;
        status->mode = recv_data.mode;
        status->last_rx_ms = vision_last_rx_ms;
        status->rx_count = vision_rx_count;
        status->tx_count = vision_tx_count;
        status->tx_drop_count = vision_tx_drop_count;
        status->crc_error_count = vision_crc_error_count;
#ifdef VISION_USE_UART
        status->uart_last_tx = vision_uart_last_tx;
#else
        memset(&status->uart_last_tx,0,sizeof(status->uart_last_tx));
#endif
        status->bullet_speed = send_data.bullet_speed;
        status->bullet_count = send_data.bullet_count;
    }
    taskEXIT_CRITICAL();
}
