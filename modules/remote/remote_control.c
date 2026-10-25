/** @file remote_control.c
* @brief 检查 18 字节 DBUS 帧，分别维护摇杆在线、键鼠更新及按下次数。
* UART 派发调用接收函数，Daemon 处理离线；命令任务周期读取当前输入和可用标记。
*/
#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_safety.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u
static RC_ctrl_t rc_ctrl[2];
static uint8_t rc_init_flag;
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;
static RemoteControlDiagnostics_s remote_control_diag;
static uint16_t pc_sequence;
static uint8_t pc_have_sequence, pc_authorized;

// 只清键鼠和按下次数，保留摇杆与开关；调用前地址必须有效。
static void ClearPC(RC_ctrl_t *input)
{
    memset(&input->mouse, 0, sizeof(input->mouse));
    memset(input->key, 0, sizeof(input->key));
    memset(input->key_count, 0, sizeof(input->key_count));
}

// 撤销键鼠使用，清心跳与两份历史输入，防止恢复后重复旧按键。
static void RevokePC(uint32_t now, RemotePcRevokeReason_e reason)
{
    if (pc_authorized)
    {
        remote_control_diag.pc_revoke_tick = now;
        remote_control_diag.pc_revoke_reason = reason;
    }
    pc_sequence = 0u; pc_have_sequence = pc_authorized = 0u;
    remote_control_diag.pc_authorized = remote_control_diag.pc_recovery_count = 0u;
    ClearPC(&rc_ctrl[TEMP]); ClearPC(&rc_ctrl[LAST]);
}

// 恢复前要求所有键、鼠标移动和鼠标按钮均为零。
static uint8_t PCReleased(const RC_ctrl_t *input)
{
    return (uint8_t)(input->key[0].keys == 0u && input->mouse.x == 0 && input->mouse.y == 0 &&
                     input->mouse.press_l == 0u && input->mouse.press_r == 0u);
}

// 超过 150 ms 没有递增心跳就清除键鼠；等于 150 ms 时仍保留。
static void ExpirePC(uint32_t now)
{
    if (pc_have_sequence && (uint32_t)(now - remote_control_diag.pc_last_advance_tick) > REMOTE_PC_LEASE_MS)
    {
        if (pc_authorized)
        {
            remote_control_diag.pc_expirations++;
            remote_control_diag.pc_expire_tick = now;
        }
        RevokePC(now, REMOTE_PC_REVOKE_TIMEOUT);
    }
}

// 调用时普通中断已关闭；只接受向前变化的心跳，恢复还要连续两帧完全松开。
static uint8_t AcceptPC(const RC_ctrl_t *candidate, uint16_t heartbeat, uint32_t now, uint32_t source_ms)
{
    ExpirePC(now);
    if (heartbeat == 0u) { RevokePC(now, REMOTE_PC_REVOKE_INACTIVE); return 0u; }
    if (!pc_authorized && !PCReleased(candidate)) { RevokePC(now, REMOTE_PC_REVOKE_INACTIVE); return 0u; }
    const uint16_t advance = (uint16_t)(heartbeat - pc_sequence);
    if (pc_have_sequence && (advance == 0u || advance >= 0x8000u)) return 0u;
    pc_sequence = heartbeat; pc_have_sequence = 1u;
    remote_control_diag.pc_heartbeat = heartbeat;
    remote_control_diag.pc_last_advance_tick = source_ms;
    if (!pc_authorized)
    {
        remote_control_diag.pc_recovery_count++;
        if (remote_control_diag.pc_recovery_count >= 2u)
            pc_authorized = 1u;
    }
    remote_control_diag.pc_authorized = pc_authorized;
    return pc_authorized;
}

// 先解码到临时对象，五个通道和开关等字段都通过后才发布，坏帧不改历史。
static uint8_t DecodeCandidate(const uint8_t *b, RC_ctrl_t *c)
{
    memset(c, 0, sizeof(*c));
    const uint16_t raw[5] = {
        (b[0] | (b[1] << 8)) & 0x07ff,
        ((b[1] >> 3) | (b[2] << 5)) & 0x07ff,
        ((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07ff,
        ((b[4] >> 1) | (b[5] << 7)) & 0x07ff,
        (b[16] | (b[17] << 8)) & 0x07ff,
    };
    for (unsigned i = 0; i < 5; ++i)
        if (raw[i] < RC_CH_VALUE_MIN || raw[i] > RC_CH_VALUE_MAX)
            return 0u;
    c->rc.switch_right = (b[5] >> 4) & 3u;
    c->rc.switch_left = (b[5] >> 6) & 3u;
    if (c->rc.switch_right == 0u || c->rc.switch_left == 0u || b[12] > 1u || b[13] > 1u)
        return 0u;
    c->rc.rocker_r_ = (int16_t)raw[0] - RC_CH_VALUE_OFFSET;
    c->rc.rocker_r1 = (int16_t)raw[1] - RC_CH_VALUE_OFFSET;
    c->rc.rocker_l_ = (int16_t)raw[2] - RC_CH_VALUE_OFFSET;
    c->rc.rocker_l1 = (int16_t)raw[3] - RC_CH_VALUE_OFFSET;
    c->rc.dial = (int16_t)raw[4] - RC_CH_VALUE_OFFSET;
    c->mouse.x = (int16_t)(b[6] | (b[7] << 8));
    c->mouse.y = (int16_t)(b[8] | (b[9] << 8));
    c->mouse.press_l = b[12]; c->mouse.press_r = b[13];
    c->key[KEY_PRESS].keys = (uint16_t)(b[14] | (b[15] << 8));
    if (c->key[KEY_PRESS].ctrl)
        c->key[KEY_PRESS_WITH_CTRL] = c->key[KEY_PRESS];
    if (c->key[KEY_PRESS].shift)
        c->key[KEY_PRESS_WITH_SHIFT] = c->key[KEY_PRESS];
    return 1u;
}

// 比较当前和上次按键，只在从未按变成按下时累加次数；Ctrl、Shift 自身不计。
static void UpdateKeyEdges(RC_ctrl_t *c)
{
    memcpy(c->key_count, rc_ctrl[TEMP].key_count, sizeof(c->key_count));
    for (unsigned i = 0; i < 16; ++i)
    {
        const uint16_t bit = (uint16_t)(1u << i);
        if (i == 4 || i == 5) continue;
        if ((c->key[KEY_PRESS].keys & bit) && !(rc_ctrl[LAST].key[KEY_PRESS].keys & bit) &&
            !(c->key[KEY_PRESS_WITH_CTRL].keys & bit) && !(c->key[KEY_PRESS_WITH_SHIFT].keys & bit))
            c->key_count[KEY_PRESS][i]++;
        // 分别累计与 Ctrl、Shift 同时按下的次数，避免算入普通按下。
        for (unsigned kind = 1; kind < 3; ++kind)
            if ((c->key[kind].keys & bit) && !(rc_ctrl[LAST].key[kind].keys & bit))
                c->key_count[kind][i]++;
    }
}

// DBUS没有帧头或CRC，只接收独立18字节块，禁止自动拼接或搜索对齐。
static void RemoteControlRxCallback(void *context, const USART_RxBlock_t *block)
{
    RC_ctrl_t *controls = context;
    if (controls == NULL || block == NULL) return;
    const uint8_t *frame = block->data;
    if (block->length != REMOTE_CONTROL_FRAME_SIZE || frame == NULL ||
        (uint32_t)(HAL_GetTick() - block->received_ms) > 50u)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        remote_control_diag.rejected_length++;
        __set_PRIMASK(primask);
        return;
    }
    RC_ctrl_t candidate;
    if (!DecodeCandidate(frame, &candidate))
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        remote_control_diag.rejected_fields++;
        __set_PRIMASK(primask);
        return;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = HAL_GetTick();
    const uint16_t heartbeat = (uint16_t)(frame[10] | ((uint16_t)frame[11] << 8));
    remote_control_diag.raw_mouse_z = heartbeat;
    remote_control_diag.raw_key_bitmap = candidate.key[0].keys;
    remote_control_diag.raw_mouse_x = (uint16_t)candidate.mouse.x;
    remote_control_diag.raw_mouse_y = (uint16_t)candidate.mouse.y;
    remote_control_diag.raw_mouse_left = candidate.mouse.press_l;
    remote_control_diag.raw_mouse_right = candidate.mouse.press_r;
    // 键鼠尚未通过检查时沿用已保存结果；撤销路径会先把该结果清零，摇杆仍可更新。
    if (AcceptPC(&candidate, heartbeat, now, block->received_ms)) UpdateKeyEdges(&candidate);
    else
    {
        candidate.mouse = controls[TEMP].mouse;
        memcpy(candidate.key, controls[TEMP].key, sizeof(candidate.key));
        memcpy(candidate.key_count, controls[TEMP].key_count, sizeof(candidate.key_count));
    }
    controls[TEMP] = candidate;
    controls[LAST] = candidate;
    remote_control_diag.accepted_frames++;
    remote_control_diag.last_valid_tick = block->received_ms;
    remote_control_diag.has_valid_frame = 1u;
    (void)DaemonReportAt(rc_daemon_instance, block->received_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
    remote_control_diag.online = 1u;
    RobotSafetySetRcOnline(1u);
    __set_PRIMASK(primask);
}

// 离线时撤销键鼠并清空摇杆，记录断开时间、请求急停，再重新准备 UART 接收。
static void RCLostCallback(const DaemonEvent_t *event, void *id)
{
    (void)id;
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    const uint32_t now = HAL_GetTick();
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(now - remote_control_diag.last_valid_tick) <= 100u)
    {
        // 新帧可能在超时检查后到达；过期回调不能撤销这份新输入。
        __set_PRIMASK(primask);
        return;
    }
    if (remote_control_diag.has_valid_frame == 0u && remote_control_diag.online == 0u)
    {
        __set_PRIMASK(primask);
        return;
    }
    RevokePC(now, REMOTE_PC_REVOKE_RADIO_LOST);
    remote_control_diag.offline_tick = now;
    remote_control_diag.offline_last_valid_tick = remote_control_diag.last_valid_tick;
    remote_control_diag.offline_events++;
    remote_control_diag.has_valid_frame = remote_control_diag.online = 0u;
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    RobotSafetySetRcOnline(0u);
    __set_PRIMASK(primask);
    EstopRequest(ESTOP_REASON_RC_OFFLINE);
    USARTServiceInit(rc_usart_instance);
    BSP_LOG_EVENT_0(EV_MODULES_REMOTE_REMOTE_CONTROL_C_01);
}

const RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *handle)
{
    if (handle == NULL)
        return NULL;
    USART_Init_Config_s conf = {
        .module_callback = RemoteControlRxCallback,
        .context = rc_ctrl,
        .usart_handle = handle,
        .recv_buff_size = REMOTE_CONTROL_FRAME_SIZE,
    };
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    memset((void *)&remote_control_diag, 0, sizeof(remote_control_diag));
    pc_authorized = 0u;
    RevokePC(HAL_GetTick(), REMOTE_PC_REVOKE_NONE);
    Daemon_Init_Config_s daemon_conf = {
        .transport_timeout_ms = 100u,
        .data_timeout_ms = 100u,
        .startup_grace_ms = 1000u,
        .event_max_age_ms = 500u,
        .source_id = 0x7001u,
        .event_capacity = 1u,
        .consumer_id = DAEMON_CONSUMER_ROBOT_TASK,
        .event_policy = DAEMON_EVENT_COALESCED_STATE,
        .consumer_mode = DAEMON_CONSUMER_BOUND_TASK,
        .event_handler = RCLostCallback,
        .owner_id = NULL,
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
    if (rc_daemon_instance == NULL)
        return NULL;
    rc_usart_instance = USARTRegister(&conf);
    // 串口注册失败时撤销已注册的离线计时对象，避免留下无主回调。
    if (rc_usart_instance == NULL)
    {
        (void)DaemonUnregister(rc_daemon_instance);
        rc_daemon_instance = NULL;
        return NULL;
    }
    RobotSafetySetRcOnline(0u);
    rc_init_flag = 1u;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t check_time = HAL_GetTick();
    const uint8_t online = (uint8_t)(rc_init_flag != 0u &&
        remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(check_time - remote_control_diag.last_valid_tick) <= 100u &&
        DaemonIsOnline(rc_daemon_instance));
    remote_control_diag.online = online;
    __set_PRIMASK(primask);
    return online;
}

void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot)
{
    if (snapshot == NULL) return;
    (void)RemoteControlIsOnline();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now = HAL_GetTick();
    const uint8_t online = (uint8_t)(rc_init_flag != 0u &&
        remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(now - remote_control_diag.last_valid_tick) <= 100u &&
        DaemonIsOnline(rc_daemon_instance));
    remote_control_diag.online = online;
    ExpirePC(now);
    snapshot->remote_online = (uint8_t)(online != 0u &&
        remote_control_diag.has_valid_frame != 0u && rc_init_flag != 0u);
    snapshot->input = rc_ctrl[TEMP];
    snapshot->pc_valid = (uint8_t)(snapshot->remote_online && pc_authorized);
    __set_PRIMASK(primask);
}

uint8_t RemoteControlGetDiagnostics(RemoteControlDiagnostics_s *output)
{
    if (output == NULL)
        return 0u;

    RemoteControlDiagnostics_s candidate;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    candidate = remote_control_diag;
    __set_PRIMASK(primask);
    *output = candidate;
    return 1u;
}
