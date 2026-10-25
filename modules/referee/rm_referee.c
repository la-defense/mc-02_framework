/**
 * @file rm_referee.c
 * @brief 校验裁判帧、按官方字段解码，并发布带独立期限的命令数据。
 * RefereeInit只注册接收；读取者通过RefereeGetSnapshot取得一致副本。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 */
#include "rm_referee.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "referee_protocol.h"
#include "crc.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "referee_tx_private.h"

#define RE_RX_BUFFER_SIZE 255u
#define REFEREE_CANDIDATE_SIZE 128u
#define REFEREE_FRAME_OVERHEAD 9u
#define REFEREE_ASSEMBLY_TIMEOUT_MS 50u

static RefereeTxBudget referee_tx_budget;
static uint8_t referee_submit_busy;
static USARTInstance *referee_usart_instance;
static DaemonInstance *referee_daemon;
static RefereeConfig_t referee_config;
static RefereeSnapshot_t referee_published;
static uint8_t referee_initialized;
static uint8_t referee_has_valid_frame;
static uint8_t referee_link_active;
static uint32_t referee_last_valid_rx_ms;

typedef struct {
    uint8_t bytes[REFEREE_CANDIDATE_SIZE];
    uint32_t time[REFEREE_CANDIDATE_SIZE];
    size_t used, expected;
    uint32_t sequence;
    uint8_t have_sequence;
    volatile uint8_t reset_requested;
} RefereeStream;
static RefereeStream referee_stream;

typedef struct { uint16_t wire_id; uint8_t payload_length; } RefereeCommandWire_t;

static const RefereeCommandWire_t referee_wire[REFEREE_COMMAND_COUNT] = {
    {ID_game_state, LEN_game_state}, {ID_game_result, LEN_game_result},
    {ID_game_robot_survivors, LEN_game_robot_HP}, {ID_event_data, LEN_event_data},
    {ID_referee_warning, LEN_referee_warning}, {ID_dart_shoot_data, LEN_dart_shoot_data},
    {ID_game_robot_state, LEN_game_robot_state}, {ID_power_heat_data, LEN_power_heat_data},
    {ID_game_robot_pos, LEN_game_robot_pos}, {ID_buff_musk, LEN_buff_musk},
    {ID_robot_hurt, LEN_robot_hurt}, {ID_shoot_data, LEN_shoot_data},
    {ID_projectile_allowance, LEN_projectile_allowance}, {ID_rfid_status, LEN_rfid_status},
    {ID_dart_client_cmd, LEN_dart_client_cmd}, {ID_ground_robot_position, LEN_ground_robot_position},
    {ID_radar_mark_data, LEN_radar_mark_data}, {ID_sentry_info, LEN_sentry_info},
    {ID_radar_info, LEN_radar_info}, {ID_map_command, LEN_map_command},
    {ID_student_interactive, LEN_receive_data}
};

static uint16_t ReadU16LE(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8u);
}

static uint32_t ReadU32LE(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

static uint64_t ReadU64LE(const uint8_t *p)
{
    uint64_t value = 0u;
    // 从高字节向低字节累积，避免依赖CPU的字节顺序。
    for (size_t i = 0u; i < 8u; ++i) value = (value << 8u) | p[7u-i];
    return value;
}

static int16_t ReadI16LE(const uint8_t *p)
{
    uint16_t raw = ReadU16LE(p);
    int32_t value = (raw & 0x8000u) ? (int32_t)raw - 65536 : (int32_t)raw;
    return (int16_t)value;
}

static float ReadF32LE(const uint8_t *p)
{
    uint32_t bits = ReadU32LE(p);
    float value;
    // 整数和浮点变量在本机使用同一种字节顺序，复制位形后再检查有限值。
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint8_t RefereeIsLegalRobotId(uint16_t id)
{ return RefereeTxRobotId(id); }

static uint8_t RefereeSameTeam(uint16_t a, uint16_t b)
{
    return (uint8_t)((a < 100u) == (b < 100u));
}

static int RefereeFindCommand(uint16_t wire_id)
{
    for (size_t i = 0u; i < REFEREE_COMMAND_COUNT; ++i)
        if (referee_wire[i].wire_id == wire_id) return (int)i;
    return -1;
}

static uint8_t RefereeDecode(int index, const uint8_t *p, size_t length,
                             RefereeCommandData_t *out, uint32_t now_ms)
{
    if (index < 0 || index >= REFEREE_COMMAND_COUNT || p == NULL || out == NULL ||
        length != referee_wire[index].payload_length) return 0u;
    RefereeCommandData_t candidate = {0};
    switch ((RefereeCommandIndex_t)index) {
    case REF_CMD_GAME_STATE:
        candidate.game_state.game_type = p[0] & REFEREE_GAME_TYPE_MASK;
        candidate.game_state.game_progress = p[0] >> REFEREE_GAME_PROGRESS_SHIFT;
        candidate.game_state.stage_remain_s = ReadU16LE(p+1u);
        candidate.game_state.sync_timestamp = ReadU64LE(p+3u);
        break;
    case REF_CMD_GAME_RESULT:
        candidate.game_result.winner = p[0];
        if (candidate.game_result.winner > 2u) return 0u;
        break;
    case REF_CMD_ROBOT_HP:
        for (size_t i = 0u; i < 4u; ++i) candidate.robot_hp.hp[i] = ReadU16LE(p+2u*i);
        candidate.robot_hp.damage_difference = ReadI16LE(p+8u);
        for (size_t i = 0u; i < 5u; ++i) candidate.robot_hp.hp[i+4u] = ReadU16LE(p+10u+2u*i);
        break;
    case REF_CMD_EVENT_DATA:
        candidate.event_data.event_bits = ReadU32LE(p);
        break;
    case REF_CMD_WARNING:
        candidate.warning.level = p[0]; candidate.warning.offending_robot_id = p[1]; candidate.warning.count = p[2];
        break;
    case REF_CMD_DART_INFO:
        candidate.dart_info.remaining_s = p[0]; candidate.dart_info.info_bits = ReadU16LE(p+1u);
        break;
    case REF_CMD_ROBOT_STATE:
        candidate.robot_state.robot_id = p[0]; candidate.robot_state.robot_level = p[1];
        candidate.robot_state.current_hp = ReadU16LE(p+2u); candidate.robot_state.maximum_hp = ReadU16LE(p+4u);
        candidate.robot_state.cooling_per_s = ReadU16LE(p+6u); candidate.robot_state.heat_limit = ReadU16LE(p+8u);
        candidate.robot_state.chassis_power_limit_w = ReadU16LE(p+10u);
        candidate.robot_state.bullet_speed_limit_mps = ReadF32LE(p+12u);
        candidate.robot_state.power_output_bits = p[16] &
            (REFEREE_OUTPUT_GIMBAL_MASK | REFEREE_OUTPUT_CHASSIS_MASK | REFEREE_OUTPUT_SHOOTER_MASK);
        if (!RefereeIsLegalRobotId(candidate.robot_state.robot_id) ||
            !isfinite(candidate.robot_state.bullet_speed_limit_mps)) return 0u;
        break;
    case REF_CMD_POWER_HEAT:
        memcpy(candidate.power_heat.reserved, p, sizeof(candidate.power_heat.reserved));
        candidate.power_heat.buffer_energy_j = ReadU16LE(p+8u);
        candidate.power_heat.heat_17mm = ReadU16LE(p+10u); candidate.power_heat.heat_42mm = ReadU16LE(p+12u);
        break;
    case REF_CMD_ROBOT_POSITION:
        candidate.robot_position.x_m = ReadF32LE(p); candidate.robot_position.y_m = ReadF32LE(p+4u);
        candidate.robot_position.angle_deg = ReadF32LE(p+8u);
        if (!isfinite(candidate.robot_position.x_m) || !isfinite(candidate.robot_position.y_m) ||
            !isfinite(candidate.robot_position.angle_deg)) return 0u;
        break;
    case REF_CMD_BUFF:
        candidate.buff.recovery_buff = p[0]; candidate.buff.cooling_buff = ReadU16LE(p+1u);
        candidate.buff.defence_buff = p[3]; candidate.buff.vulnerability_buff = p[4];
        candidate.buff.attack_buff = ReadU16LE(p+5u); candidate.buff.remaining_energy_bits = p[7];
        break;
    case REF_CMD_ROBOT_HURT:
        candidate.robot_hurt.armor_id = p[0] & 0x0fu;
        candidate.robot_hurt.hurt_type = p[0] >> 4u;
        if (candidate.robot_hurt.hurt_type != 0u && candidate.robot_hurt.hurt_type != 1u &&
            candidate.robot_hurt.hurt_type != 5u) return 0u;
        break;
    case REF_CMD_SHOOT_DATA:
        candidate.shoot_data.bullet_type = p[0]; candidate.shoot_data.shooter_number = p[1];
        candidate.shoot_data.frequency_hz = p[2]; candidate.shoot_data.initial_speed_mps = ReadF32LE(p+3u);
        if (!isfinite(candidate.shoot_data.initial_speed_mps)) return 0u;
        break;
    case REF_CMD_PROJECTILE_ALLOWANCE:
        candidate.projectile_allowance.allowance_17mm = ReadU16LE(p);
        candidate.projectile_allowance.allowance_42mm = ReadU16LE(p+2u);
        candidate.projectile_allowance.remaining_gold = ReadU16LE(p+4u);
        candidate.projectile_allowance.allowance_fortress = ReadU16LE(p+6u);
        break;
    case REF_CMD_RFID:
        candidate.rfid.status_bits = ReadU32LE(p); candidate.rfid.extra_status = p[4];
        break;
    case REF_CMD_DART_CLIENT:
        candidate.dart_client.opening_status = p[0]; candidate.dart_client.reserved = p[1];
        candidate.dart_client.target_change_time = ReadU16LE(p+2u);
        candidate.dart_client.latest_launch_time = ReadU16LE(p+4u);
        if (candidate.dart_client.opening_status > 2u) return 0u;
        break;
    case REF_CMD_GROUND_POSITION:
        for (size_t i = 0u; i < 8u; ++i) {
            candidate.ground_position.position_m[i] = ReadF32LE(p+4u*i);
            if (!isfinite(candidate.ground_position.position_m[i])) return 0u;
        }
        memcpy(candidate.ground_position.reserved, p+32u, sizeof(candidate.ground_position.reserved));
        break;
    case REF_CMD_RADAR_MARK:
        candidate.radar_mark.mark_bits = ReadU16LE(p);
        break;
    case REF_CMD_SENTRY_INFO:
        candidate.sentry_info.info_bits_1 = ReadU32LE(p);
        candidate.sentry_info.info_bits_2 = ReadU16LE(p+4u);
        candidate.sentry_info.info_bits_3 = ReadU64LE(p+6u);
        break;
    case REF_CMD_RADAR_INFO:
        candidate.radar_info.info_bits = p[0];
        break;
    case REF_CMD_MAP_COMMAND:
        candidate.map_command.target_x_m = ReadF32LE(p); candidate.map_command.target_y_m = ReadF32LE(p+4u);
        candidate.map_command.keyboard_bits = p[8]; candidate.map_command.target_robot_id = p[9];
        candidate.map_command.source_id = ReadU16LE(p+10u);
        if (!isfinite(candidate.map_command.target_x_m) || !isfinite(candidate.map_command.target_y_m)) return 0u;
        break;
    case REF_CMD_INTERACTIVE: {
        candidate.interactive.content_id = ReadU16LE(p);
        candidate.interactive.sender_id = ReadU16LE(p+2u);
        candidate.interactive.receiver_id = ReadU16LE(p+4u);
        memcpy(candidate.interactive.data, p+6u, sizeof(candidate.interactive.data));
        if (candidate.interactive.content_id != Communicate_Data_ID ||
            !RefereeIsLegalRobotId(candidate.interactive.sender_id) ||
            !RefereeIsLegalRobotId(candidate.interactive.receiver_id) ||
            !RefereeSameTeam(candidate.interactive.sender_id, candidate.interactive.receiver_id)) return 0u;
        uint16_t own_id = 0u;
        taskENTER_CRITICAL();
        const RefereeCommandSnapshot_t *robot = &referee_published.command[REF_CMD_ROBOT_STATE];
        if (robot->receive_count != 0u &&
            (uint32_t)(now_ms-robot->source_rx_ms) <= referee_config.timeout_10hz_ms)
            own_id = robot->data.robot_state.robot_id;
        taskEXIT_CRITICAL();
        if (own_id == 0u || candidate.interactive.receiver_id != own_id ||
            !RefereeIsLegalRobotId(own_id) || !RefereeSameTeam(own_id, candidate.interactive.sender_id)) return 0u;
        break;
    }
    case REFEREE_COMMAND_COUNT:
    default:
        return 0u;
    }
    *out = candidate;
    return 1u;
}

static uint32_t RefereeTimeoutForIndex(size_t index)
{
    switch ((RefereeCommandIndex_t)index) {
    case REF_CMD_ROBOT_STATE: case REF_CMD_POWER_HEAT: case REF_CMD_PROJECTILE_ALLOWANCE:
        return referee_config.timeout_10hz_ms;
    case REF_CMD_ROBOT_HP: case REF_CMD_BUFF: case REF_CMD_RFID: case REF_CMD_DART_CLIENT:
        return referee_config.timeout_3hz_ms;
    case REF_CMD_GAME_STATE: case REF_CMD_EVENT_DATA: case REF_CMD_WARNING: case REF_CMD_DART_INFO:
    case REF_CMD_ROBOT_POSITION: case REF_CMD_GROUND_POSITION: case REF_CMD_RADAR_MARK:
    case REF_CMD_SENTRY_INFO: case REF_CMD_RADAR_INFO:
        return referee_config.timeout_1hz_ms;
    case REF_CMD_MAP_COMMAND:
        return referee_config.timeout_map_ms;
    default:
        return 0u;
    }
}

static uint8_t RefereeIsEventIndex(size_t index)
{
    return (uint8_t)(index == REF_CMD_GAME_RESULT || index == REF_CMD_ROBOT_HURT ||
                     index == REF_CMD_SHOOT_DATA || index == REF_CMD_INTERACTIVE);
}

static void RefereeShift(RefereeStream *stream, size_t count)
{
    stream->used -= count;
    memmove(stream->bytes, stream->bytes+count, stream->used);
    memmove(stream->time, stream->time+count, stream->used*sizeof(stream->time[0]));
    stream->expected = 0u;
}

static uint8_t RefereePublish(int index, const RefereeCommandData_t *data, uint8_t wire_sequence,
                              uint32_t source_rx_ms, uint32_t now_ms)
{
    if (data == NULL || index < 0 || index >= REFEREE_COMMAND_COUNT) return 0u;
    taskENTER_CRITICAL();
    if (referee_stream.reset_requested || (uint32_t)(now_ms-source_rx_ms) > REFEREE_ASSEMBLY_TIMEOUT_MS) {
        taskEXIT_CRITICAL();
        return 0u;
    }
    RefereeCommandSnapshot_t *entry = &referee_published.command[index];
    entry->data = *data;
    entry->source_rx_ms = source_rx_ms;
    entry->wire_sequence = wire_sequence;
    if (entry->receive_count != UINT32_MAX) ++entry->receive_count;
    referee_last_valid_rx_ms = source_rx_ms;
    referee_has_valid_frame = 1u;
    referee_link_active = 1u;
    (void)DaemonReportAt(referee_daemon, source_rx_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
    taskEXIT_CRITICAL();
    return 1u;
}

static uint8_t RefereeAccept(const uint8_t *frame, size_t frame_length, uint32_t source_rx_ms)
{
    uint16_t wire_id = ReadU16LE(frame+CMD_ID_Offset);
    int index = RefereeFindCommand(wire_id);
    if (index < 0 || frame_length < REFEREE_FRAME_OVERHEAD) return 0u;
    size_t payload_length = frame_length-REFEREE_FRAME_OVERHEAD;
    RefereeCommandData_t candidate;
    uint32_t now_ms = HAL_GetTick();
    if (!RefereeDecode(index, frame+DATA_Offset, payload_length, &candidate, now_ms)) return 0u;
    return RefereePublish(index, &candidate, frame[3], source_rx_ms, now_ms);
}

// 每个字节都保留原接收块时间，CRC失败时移走一个字节继续找后面的帧头。
static void RefereeRxCallback(void *context, const USART_RxBlock_t *block)
{
    RefereeStream *stream = context;
    if (stream == NULL || block == NULL) return;
    taskENTER_CRITICAL();
    if (stream->reset_requested) {
        stream->used = stream->expected = 0u;
        stream->reset_requested = 0u;
    }
    taskEXIT_CRITICAL();
    uint32_t now = HAL_GetTick();
    if (block->data == NULL || block->length == 0u || block->length > RE_RX_BUFFER_SIZE) {
        stream->used = stream->expected = 0u;
        return;
    }
    if ((stream->have_sequence && block->sequence != stream->sequence+1u) ||
        (stream->used && (uint32_t)(now-stream->time[0]) > REFEREE_ASSEMBLY_TIMEOUT_MS))
        stream->used = stream->expected = 0u;
    stream->sequence = block->sequence;
    stream->have_sequence = 1u;
    if ((uint32_t)(now-block->received_ms) > REFEREE_ASSEMBLY_TIMEOUT_MS) {
        stream->used = stream->expected = 0u;
        return;
    }
    for (size_t i = 0u; i < block->length; ++i) {
        if (stream->reset_requested) return;
        stream->bytes[stream->used] = block->data[i];
        stream->time[stream->used++] = block->received_ms;
        while (stream->used != 0u) {
            if (stream->bytes[0] != REFEREE_SOF) { RefereeShift(stream, 1u); continue; }
            if (stream->used < LEN_HEADER) break;
            uint32_t crc;
            if (stream->expected == 0u) {
                if (CRCCompute(CRC8_REFEREE, stream->bytes, 4u, &crc) != CRC_OK || crc != stream->bytes[4]) {
                    RefereeShift(stream, 1u); continue;
                }
                size_t payload_length = (size_t)stream->bytes[1] | ((size_t)stream->bytes[2] << 8u);
                if (payload_length > sizeof(stream->bytes)-REFEREE_FRAME_OVERHEAD) {
                    RefereeShift(stream, 1u); continue;
                }
                stream->expected = payload_length+REFEREE_FRAME_OVERHEAD;
            }
            if (stream->used < stream->expected) break;
            size_t length = stream->expected;
            if (CRCCompute(CRC16_SP_REFEREE, stream->bytes, length-2u, &crc) != CRC_OK ||
                crc != ((uint32_t)stream->bytes[length-2u] | ((uint32_t)stream->bytes[length-1u] << 8u))) {
                RefereeShift(stream, 1u); continue;
            }
            (void)RefereeAccept(stream->bytes, length, stream->time[0]);
            RefereeShift(stream, length);
        }
    }
}

static void RefereeLostCallback(const DaemonEvent_t *event, void *arg)
{
    (void)arg;
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    taskENTER_CRITICAL();
    referee_stream.reset_requested = 1u;
    referee_link_active = 0u;
    taskEXIT_CRITICAL();
    // 只撤销链路在线标记；各命令旧值及来源时间保留供诊断。
    USARTServiceInit(referee_usart_instance);
    BSP_LOG_EVENT_0(EV_MODULES_REFEREE_RM_REFEREE_C_01);
}

static uint8_t RefereeConfigValid(const RefereeConfig_t *config)
{
    return (uint8_t)(config != NULL && config->timeout_10hz_ms != 0u && config->timeout_10hz_ms <= INT32_MAX &&
                     config->timeout_3hz_ms != 0u && config->timeout_3hz_ms <= INT32_MAX &&
                     config->timeout_1hz_ms != 0u && config->timeout_1hz_ms <= INT32_MAX &&
                     config->timeout_map_ms != 0u && config->timeout_map_ms <= INT32_MAX);
}

// 所有裁判发送者都检查当前身份；即使低层UI调用没有自定义guard，也不能发送旧身份。
static uint8_t RefereeFrameCanStart(void *context, const uint8_t *frame, size_t length)
{
    (void)context;
    return length >= 15u && frame[0] == REFEREE_SOF &&
        RefereeIdentityMatches((uint16_t)(frame[9] | ((uint16_t)frame[10] << 8u)));
}
// UART只有一个发送推进者；额度先观察当前终态，再尝试下一个队首。
static USARTTxGateResult_t RefereeTxGate(void *context, uint32_t id, const uint8_t *frame, size_t length)
{
    (void)context;
    if (length < 15u || length > REFEREE_TX_FRAME_MAX) return USART_TX_REJECT;
    taskENTER_CRITICAL();
    USARTTxGateResult_t result = RefereeBudgetGate(&referee_tx_budget, id,
        ReadU16LE(frame + 11u), (uint16_t)length, HAL_GetTick());
    taskEXIT_CRITICAL();
    return result;
}
static void RefereeTxObserve(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    taskENTER_CRITICAL();
    RefereeBudgetObserve(&referee_tx_budget, event);
    taskEXIT_CRITICAL();
}
RefereeStatus_t RefereeInit(UART_HandleTypeDef *handle, const RefereeConfig_t *config)
{
    if (handle == NULL) return REFEREE_ERROR_ARGUMENT;
    if (referee_initialized || referee_usart_instance != NULL || referee_daemon != NULL)
        return REFEREE_ERROR_ALREADY_INITIALIZED;
    const RefereeConfig_t defaults = {300u, 1000u, 3000u, 3000u};
    const RefereeConfig_t *selected = config == NULL ? &defaults : config;
    if (!RefereeConfigValid(selected)) return REFEREE_ERROR_CONFIG;

    Daemon_Init_Config_s daemon_conf = {
        .transport_timeout_ms = REFEREE_LINK_TIMEOUT_MS,
        .data_timeout_ms = REFEREE_LINK_TIMEOUT_MS,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 1000u,
        .source_id = 0x7003u,
        .event_capacity = 1u,
        .consumer_id = DAEMON_CONSUMER_MOTOR_TASK,
        .event_policy = DAEMON_EVENT_COALESCED_STATE,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
        .event_handler = RefereeLostCallback,
        .owner_id = &referee_published,
    };
    DaemonInstance *daemon = DaemonRegister(&daemon_conf);
    if (daemon == NULL) return REFEREE_ERROR_RESOURCE;
    // UART注册会立即启动DMA；先准备解析配置和daemon，避免首个块抢在初始化返回前到达。
    taskENTER_CRITICAL();
    referee_config = *selected;
    memset(&referee_published, 0, sizeof(referee_published));
    memset(&referee_stream, 0, sizeof(referee_stream));
    referee_last_valid_rx_ms = 0u;
    referee_has_valid_frame = 0u;
    referee_link_active = 0u;
    referee_daemon = daemon;
    taskEXIT_CRITICAL();
    USART_Init_Config_s uart_conf = {0};
    uart_conf.module_callback = RefereeRxCallback;
    uart_conf.context = &referee_stream;
    uart_conf.usart_handle = handle;
    uart_conf.recv_buff_size = RE_RX_BUFFER_SIZE;
    uart_conf.tx_guard = RefereeFrameCanStart;
    uart_conf.tx_gate = RefereeTxGate;
    uart_conf.tx_observer = RefereeTxObserve;
    USARTInstance *instance = USARTRegister(&uart_conf);
    if (instance == NULL) {
        taskENTER_CRITICAL();
        referee_daemon = NULL;
        memset(&referee_published, 0, sizeof(referee_published));
        memset(&referee_stream, 0, sizeof(referee_stream));
        referee_last_valid_rx_ms = 0u;
        referee_has_valid_frame = 0u;
        referee_link_active = 0u;
        taskEXIT_CRITICAL();
        (void)DaemonUnregister(daemon);
        return REFEREE_ERROR_RESOURCE;
    }
    taskENTER_CRITICAL();
    referee_usart_instance = instance;
    referee_initialized = 1u;
    taskEXIT_CRITICAL();
    return REFEREE_OK;
}

RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output)
{
    if (output == NULL) return REFEREE_ERROR_ARGUMENT;
    taskENTER_CRITICAL();
    if (!referee_initialized) {
        taskEXIT_CRITICAL();
        memset(output, 0, sizeof(*output));
        return REFEREE_ERROR_NOT_INITIALIZED;
    }
    *output = referee_published;
    uint32_t now_ms = HAL_GetTick();
    uint32_t link_rx_ms = referee_last_valid_rx_ms;
    uint8_t link_has_valid = referee_has_valid_frame;
    uint8_t link_active = referee_link_active;
    uint8_t daemon_online = (uint8_t)(referee_daemon != NULL && DaemonIsOnline(referee_daemon));
    taskEXIT_CRITICAL();

    output->has_valid_frame = link_has_valid;
    output->link_source_rx_ms = link_rx_ms;
    output->link_age_ms = link_has_valid ? (uint32_t)(now_ms-link_rx_ms) : 0u;
    output->link_online = (uint8_t)(link_active && link_has_valid &&
        output->link_age_ms <= REFEREE_LINK_TIMEOUT_MS && daemon_online);
    for (size_t i = 0u; i < REFEREE_COMMAND_COUNT; ++i) {
        RefereeCommandSnapshot_t *entry = &output->command[i];
        if (entry->receive_count == 0u) {
            entry->state = REFEREE_DATA_UNSEEN;
            entry->age_ms = 0u;
        } else {
            entry->age_ms = (uint32_t)(now_ms-entry->source_rx_ms);
            entry->state = RefereeIsEventIndex(i) ? REFEREE_DATA_EVENT_RECORDED :
                (entry->age_ms <= RefereeTimeoutForIndex(i) ? REFEREE_DATA_FRESH : REFEREE_DATA_EXPIRED);
        }
    }
    return REFEREE_OK;
}

// UI只开放现有六类；服务器自主决策等子命令不随共享调度接入。
static int RefereeContentLength(uint16_t command, size_t length)
{
    switch (command)
    {
    case 0x0100u:
        return length == 2u;
    case 0x0101u:
        return length == 15u;
    case 0x0102u:
        return length == 30u;
    case 0x0103u:
        return length == 75u;
    case 0x0104u:
        return length == 105u;
    case 0x0110u:
        return length == 45u;
    default:
        return command >= 0x0200u && command <= 0x02ffu && length <= REFEREE_TX_CONTENT_MAX;
    }
}
static void TxWriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8u);
}
RefereeSendStatus_t RefereeSubmit(const RefereeTxRequest_t *request, uint32_t *id)
{
    if (request == NULL || id == NULL || request->complete == NULL ||
        (request->content == NULL && request->length != 0u) || request->timeout_ms == 0u ||
        request->timeout_ms > INT32_MAX || !RefereeContentLength(request->content_id, request->length))
    {
        taskENTER_CRITICAL();
        ++referee_tx_budget.state.validation_rejected;
        ++referee_tx_budget.state.rejected;
        taskEXIT_CRITICAL();
        return REFEREE_SEND_BAD_ARGUMENT;
    }
    taskENTER_CRITICAL();
    if (!referee_initialized)
    {
        ++referee_tx_budget.state.rejected;
        taskEXIT_CRITICAL();
        return REFEREE_SEND_NOT_INITIALIZED;
    }
    const RefereeCommandSnapshot_t *robot = &referee_published.command[REF_CMD_ROBOT_STATE];
    uint16_t sender = robot->data.robot_state.robot_id;
    uint8_t fresh = robot->receive_count != 0u && (uint32_t)(HAL_GetTick() - robot->source_rx_ms) <=
                                                      RefereeTimeoutForIndex(REF_CMD_ROBOT_STATE);
    uint16_t receiver = request->receiver_id;
    uint8_t destination;
    if (request->content_id >= 0x0200u)
    {
        destination = RefereeTxRobotId(receiver) && RefereeSameTeam(sender, receiver);
    }
    else
    {
        uint16_t client = receiver >= 0x100u ? (uint16_t)(receiver - 0x100u) : 0u;
        destination = ((client >= 1u && client <= 6u) || (client >= 101u && client <= 106u)) &&
                      RefereeSameTeam(sender, client) && (sender == client || sender == 9u || sender == 109u);
    }
    if (!fresh || !RefereeTxRobotId(sender) || !destination)
    {
        ++referee_tx_budget.state.validation_rejected;
        ++referee_tx_budget.state.rejected;
        taskEXIT_CRITICAL();
        return REFEREE_SEND_IDENTITY;
    }
    // 不在临界区内组帧或调用UART；并发提交争用立即拒绝，成功后才消耗序号。
    if (referee_submit_busy)
    {
        ++referee_tx_budget.state.rejected;
        taskEXIT_CRITICAL();
        return REFEREE_SEND_BUSY;
    }
    referee_submit_busy = 1u;
    uint8_t sequence = referee_tx_budget.state.next_sequence;
    taskEXIT_CRITICAL();
    uint8_t frame[REFEREE_TX_FRAME_MAX] = {0};
    size_t total = 15u + request->length;
    frame[0] = REFEREE_SOF;
    frame[3] = sequence;
    TxWriteU16(frame + 1u, (uint16_t)(6u + request->length));
    TxWriteU16(frame + 5u, ID_student_interactive);
    TxWriteU16(frame + 7u, request->content_id);
    TxWriteU16(frame + 9u, sender);
    TxWriteU16(frame + 11u, receiver);
    if (request->length)
        memcpy(frame + 13u, request->content, request->length);
    uint32_t crc;
    RefereeSendStatus_t result = REFEREE_SEND_CRC;
    if (CRCCompute(CRC8_REFEREE, frame, 4u, &crc) == CRC_OK)
    {
        frame[4] = (uint8_t)crc;
        if (CRCCompute(CRC16_SP_REFEREE, frame, total - 2u, &crc) == CRC_OK)
        {
            TxWriteU16(frame + total - 2u, (uint16_t)crc);
            USARTTxRequest_t uart_request = {
                frame,          total,           USART_TRANSFER_DMA, request->timeout_ms, request->complete,
                request->guard, request->context};
            switch (USARTSubmitTx(referee_usart_instance, &uart_request, id))
            {
            case USART_SEND_ACCEPTED:
                result = REFEREE_SEND_ACCEPTED;
                break;
            case USART_SEND_BUSY:
                result = REFEREE_SEND_BUSY;
                break;
            case USART_SEND_NOT_READY:
                result = REFEREE_SEND_NOT_INITIALIZED;
                break;
            case USART_SEND_ISOLATED:
                result = REFEREE_SEND_ISOLATED;
                break;
            default:
                result = REFEREE_SEND_ERROR;
                break;
            }
        }
    }
    taskENTER_CRITICAL();
    if (result == REFEREE_SEND_ACCEPTED)
        ++referee_tx_budget.state.next_sequence;
    else
        ++referee_tx_budget.state.rejected;
    referee_submit_busy = 0u;
    taskEXIT_CRITICAL();
    return result;
}
USARTSendStatus_t RefereeCancelSend(uint32_t id)
{
    return USARTCancelTx(referee_usart_instance, id);
}
USARTSendStatus_t RefereeGetTxState(RefereeTxState_t *output)
{
    if (output == NULL)
        return USART_SEND_BAD_ARGUMENT;
    if (referee_usart_instance == NULL)
        return USART_SEND_NOT_READY;
    USARTTxState_t uart;
    USARTSendStatus_t result = USARTGetTxState(referee_usart_instance, &uart);
    if (result != USART_SEND_ACCEPTED)
        return result;
    taskENTER_CRITICAL();
    *output = referee_tx_budget.state;
    output->uart = uart;
    if (uart.used == 0u)
        output->wait_reason = REFEREE_TX_WAIT_NONE;
    taskEXIT_CRITICAL();
    return USART_SEND_ACCEPTED;
}

uint8_t RefereeIdentityMatches(uint16_t robot_id)
{
    taskENTER_CRITICAL();
    const RefereeCommandSnapshot_t *entry=&referee_published.command[REF_CMD_ROBOT_STATE];
    uint8_t matches=(uint8_t)(referee_usart_instance!=NULL && entry->receive_count!=0u &&
        (uint32_t)(HAL_GetTick()-entry->source_rx_ms)<=RefereeTimeoutForIndex(REF_CMD_ROBOT_STATE) &&
        entry->data.robot_state.robot_id==robot_id);
    taskEXIT_CRITICAL();return matches;
}
