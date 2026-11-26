/** @file remote_control.c
* @brief 检查 18 字节 DBUS 帧，分别维护摇杆在线、键鼠更新及按下次数。
* UART 派发调用接收函数，Daemon 处理离线；命令任务周期读取当前输入和可用标记。
*/
#include "remote_control.h"
#include "remote_safety.h"
#include "string.h"
#include "platform_uart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_safety.h"
#include "platform_irq.h"
#include "platform_sync.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u
#define REMOTE_RX_DISPATCH_MAX_AGE_MS 50u // 从 UART 原始接收到提交遥控状态的最长间隔。
static RC_ctrl_t rc_ctrl[2];
static uint8_t rc_init_flag;
static PlatformUartInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;
static RemoteControlDiagnostics_s remote_control_diag;
static uint16_t pc_sequence;
static uint8_t pc_have_sequence, pc_authorized;
static uint32_t radio_sample_sequence; // 每份合法无线输入递增，独立于诊断累计数。
static uint32_t source_selection_generation; // 记录来源开关变化和断线重连。
static uint32_t gesture_switch_generation; // 记录开关、拨轮急停和断线重连的变化，为持续手势提供连续性依据。
static uint32_t pc_sample_sequence; // 只有键鼠心跳向前并接受时递增，超时不清去重历史。
typedef struct {
    int16_t x, y; // 一次接受的鼠标相对位移。
    uint32_t source_ms; // UART 接收到该位移的原始时间。
    uint32_t sequence; // 该位移所属的本机键鼠样本号。
} RemoteMouseEvent_t;
static RemoteMouseEvent_t mouse_events[REMOTE_MOUSE_EVENT_CAPACITY]; // 按到达顺序保存未消费位移。
static uint8_t mouse_head, mouse_count, mouse_overflow; // 环形队列位置、数量和溢出状态。

// 调用前已进入中断保护；撤销或切换输入时不能保留旧位移。
static void ClearMousePending(void)
{
    mouse_head = 0u; // 下一次入队从队首开始。
    mouse_count = 0u; // 丢弃全部旧事件。
    mouse_overflow = 0u; // 新的输入阶段重新开始统计。
}

// 调用前已进入中断保护；队列满时整批丢弃并等待消费者确认。
static void QueueMouse(const RC_ctrl_t *input, uint32_t source_ms)
{
    if (!switch_is_up(input->rc.switch_left))
    {
        ClearMousePending(); // 切出键鼠档后不能再执行之前的位移。
        return;
    }
    if (input->mouse.x == 0 && input->mouse.y == 0) return; // 零位移不占用事件槽。
    if (mouse_overflow != 0u) return; // 消费者尚未观察到溢出前不接受不完整的新批次。
    if (mouse_count == REMOTE_MOUSE_EVENT_CAPACITY)
    {
        mouse_count = 0u; // 丢弃无法完整交付的积压。
        mouse_head = 0u; // 复位索引，等待消费者确认溢出。
        mouse_overflow = 1u; // 本批不能被误当成完整位移。
        if (remote_control_diag.mouse_overflows != UINT32_MAX) ++remote_control_diag.mouse_overflows; // 诊断计数到上限后保持。
        return;
    }
    const uint8_t tail = (uint8_t)((mouse_head + mouse_count) % REMOTE_MOUSE_EVENT_CAPACITY); // 在队尾保存新事件。
    mouse_events[tail] = (RemoteMouseEvent_t){input->mouse.x, input->mouse.y, source_ms, pc_sample_sequence}; // 保存原始时间与样本号。
    ++mouse_count; // 完成复制后再增加可读数量。
}

// 保留0为未收到；32位自然回绕时跳过0，保证使用方能识别首份有效输入。
static uint32_t NextSampleSequence(uint32_t previous)
{
    const uint32_t next = previous + 1u; // 无符号加法允许从最大值回到0。
    return next == 0u ? 1u : next; // 0不交付为合法样本编号。
}

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
        if (switch_is_up(rc_ctrl[TEMP].rc.switch_left))
            RobotSafetyRevokeManualSource(); // 已选键鼠失效时立即撤销，摇杆档的后台键鼠故障不影响其许可。
    }
    pc_sequence = 0u; pc_have_sequence = pc_authorized = 0u;
    ClearMousePending(); // 失联或人工撤销后不能重放旧鼠标位移。
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
    pc_sample_sequence = NextSampleSequence(pc_sample_sequence); // 只给这次接受的新键鼠数据分配本机编号。
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
        (uint32_t)(PlatformTimeNowMs() - block->received_ms) > REMOTE_RX_DISPATCH_MAX_AGE_MS)
    {
        const uint32_t primask = PlatformIrqSaveAndDisable();
        remote_control_diag.rejected_length++;
        PlatformIrqRestore(primask);
        return;
    }
    RC_ctrl_t candidate;
    if (!DecodeCandidate(frame, &candidate))
    {
        const uint32_t primask = PlatformIrqSaveAndDisable();
        remote_control_diag.rejected_fields++;
        PlatformIrqRestore(primask);
        return;
    }
    uint32_t primask = PlatformIrqSaveAndDisable();
    uint32_t now = PlatformTimeNowMs(); // 解码期间可能被抢占，提交前重新取时。
    if ((uint32_t)(now - block->received_ms) > REMOTE_RX_DISPATCH_MAX_AGE_MS)
    {
        ++remote_control_diag.rejected_length; // 沿用超出派发期限的诊断项。
        PlatformIrqRestore(primask); // 不改写档位、按键或来源序号。
        return;
    }
    if (remote_control_diag.has_valid_frame != 0u)
    {
        const uint32_t gap = block->received_ms - remote_control_diag.last_valid_tick; // 比较实际接收时刻，不用延迟派发时刻。
        if (gap > INT32_MAX)
        {
            ++remote_control_diag.rejected_fields; // 迟到的旧无线帧不能覆盖较新状态。
            PlatformIrqRestore(primask);
            return;
        }
        if (gap > REMOTE_RADIO_LEASE_MS)
        {
            RevokePC(now, REMOTE_PC_REVOKE_RADIO_LOST); // 离线回调尚未执行时也清除旧位移并要求重新授权。
            remote_control_diag.offline_tick = now; // 保留本次发现断联的时刻。
            remote_control_diag.offline_last_valid_tick = remote_control_diag.last_valid_tick; // 保存断联前最后接收时间。
            ++remote_control_diag.offline_events; // 与离线回调使用同一诊断计数。
            remote_control_diag.has_valid_frame = remote_control_diag.online = 0u; // 避免延迟离线回调重复处理。
            RobotSafetySetRcOnline(0u); // 先在保护内撤销旧无线许可。
            PlatformIrqRestore(primask); // 急停日志不能放在外层中断屏蔽中提交。
            if (!RobotSafetyIsFormalAutonomous()) EstopRequest(ESTOP_REASON_RC_OFFLINE); // 调试和手动模式锁存急停。
            primask = PlatformIrqSaveAndDisable(); // USART 单一派发者尚未返回；重新保护状态后处理本帧。
            now = PlatformTimeNowMs(); // 重新取时，避免中途抢占后接受过期帧。
            if ((uint32_t)(now - block->received_ms) > REMOTE_RX_DISPATCH_MAX_AGE_MS)
            {
                ++remote_control_diag.rejected_length; // 已过接收处理期限的帧不重新授权。
                PlatformIrqRestore(primask);
                return;
            }
        }
    }
    const uint16_t heartbeat = (uint16_t)(frame[10] | ((uint16_t)frame[11] << 8));
    remote_control_diag.raw_mouse_z = heartbeat;
    remote_control_diag.raw_key_bitmap = candidate.key[0].keys;
    remote_control_diag.raw_mouse_x = (uint16_t)candidate.mouse.x;
    remote_control_diag.raw_mouse_y = (uint16_t)candidate.mouse.y;
    remote_control_diag.raw_mouse_left = candidate.mouse.press_l;
    remote_control_diag.raw_mouse_right = candidate.mouse.press_r;
    // 键鼠尚未通过检查时沿用已保存结果；撤销路径会先把该结果清零，摇杆仍可更新。
    if (AcceptPC(&candidate, heartbeat, now, block->received_ms))
    {
        UpdateKeyEdges(&candidate); // 只有新心跳才更新按键边沿。
        QueueMouse(&candidate, block->received_ms); // 位移单独排队，避免多帧被最新值覆盖。
    }
    else
    {
        candidate.mouse = controls[TEMP].mouse;
        memcpy(candidate.key, controls[TEMP].key, sizeof(candidate.key));
        memcpy(candidate.key_count, controls[TEMP].key_count, sizeof(candidate.key_count));
    }
    if (!switch_is_up(candidate.rc.switch_left)) ClearMousePending(); // 即使 PC 心跳重复，切出键鼠档也要清除旧位移。
    const uint8_t source_switched = (uint8_t)(remote_control_diag.has_valid_frame != 0u &&
        controls[TEMP].rc.switch_left != candidate.rc.switch_left); // 已取得合法输入后的切档才是来源变化。
    const uint8_t dial_asserted = (uint8_t)(candidate.rc.dial > REMOTE_ESTOP_DIAL_THRESHOLD); // 合法帧的拨轮急停不等待命令任务轮询。
    const uint8_t log_dial = EstopAcceptRemoteDial(dial_asserted); // 同一保护内锁存急停，再公开遥控输入。
    if (remote_control_diag.has_valid_frame == 0u || source_switched)
        source_selection_generation = NextSampleSequence(source_selection_generation); // 合法来源切换即使很短也留下记录。
    if (remote_control_diag.has_valid_frame == 0u || source_switched ||
        controls[TEMP].rc.switch_right != candidate.rc.switch_right ||
        (controls[TEMP].rc.dial > REMOTE_ESTOP_DIAL_THRESHOLD) != dial_asserted)
        gesture_switch_generation = NextSampleSequence(gesture_switch_generation); // 开关变化及拨轮触发或释放都会结束上一段手势。
    if (source_switched)
        RobotSafetyRevokeManualSource(); // 接收端先撤销旧人工许可，命令任务稍后用新来源接续。
    controls[TEMP] = candidate;
    controls[LAST] = candidate;
    remote_control_diag.accepted_frames++;
    radio_sample_sequence = NextSampleSequence(radio_sample_sequence); // 整帧检查通过后才推进无线样本号。
    remote_control_diag.last_valid_tick = block->received_ms;
    remote_control_diag.has_valid_frame = 1u;
    (void)DaemonReportAt(rc_daemon_instance, block->received_ms,
        DAEMON_REPORT_TRANSPORT_VALID | DAEMON_REPORT_DATA_PRESENT | DAEMON_REPORT_DATA_VALID);
    remote_control_diag.online = 1u;
    RobotSafetySetRcOnline(1u);
    PlatformIrqRestore(primask);
    if (log_dial)
        EstopReportRemoteDialLatched(); // 退出接收保护后才提交急停日志。
}

// 离线时撤销键鼠并清空摇杆，记录断开时间、请求急停，再重新准备 UART 接收。
static void RCLostCallback(const DaemonEvent_t *event, void *id)
{
    (void)id;
    if (event == NULL || event->state != DAEMON_SOURCE_OFFLINE)
        return;
    const uint32_t now = PlatformTimeNowMs();
    const uint32_t primask = PlatformIrqSaveAndDisable();
    if (remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(now - remote_control_diag.last_valid_tick) <= REMOTE_RADIO_LEASE_MS)
    {
        // 新帧可能在超时检查后到达；过期回调不能撤销这份新输入。
        PlatformIrqRestore(primask);
        return;
    }
    if (remote_control_diag.has_valid_frame == 0u && remote_control_diag.online == 0u)
    {
        PlatformIrqRestore(primask);
        return;
    }
    RevokePC(now, REMOTE_PC_REVOKE_RADIO_LOST);
    remote_control_diag.offline_tick = now;
    remote_control_diag.offline_last_valid_tick = remote_control_diag.last_valid_tick;
    remote_control_diag.offline_events++;
    remote_control_diag.has_valid_frame = remote_control_diag.online = 0u;
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    RobotSafetySetRcOnline(0u);
    PlatformIrqRestore(primask);
    if (!RobotSafetyIsFormalAutonomous()) EstopRequest(ESTOP_REASON_RC_OFFLINE); // 正式自主按比赛与核心条件运行，其他模式离线急停。
    PlatformUartRestartReceive(rc_usart_instance);
    BSP_LOG_EVENT_0(EV_MODULES_REMOTE_REMOTE_CONTROL_C_01);
}

const RC_ctrl_t *RemoteControlInit(PlatformUartPort_t port)
{
    if (!PlatformUartIsBound(port))
        return NULL; // 未绑定时不注册来源或启动 DMA。
    PlatformUartConfig conf = {
        .receive = RemoteControlRxCallback, // 按原规则检查收到的 DBUS 帧。
        .context = rc_ctrl, // 接收回调使用长期保存的遥控数据。
        .receive_capacity = REMOTE_CONTROL_FRAME_SIZE, // 每块最多接收 18 字节。
    };
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    memset((void *)&remote_control_diag, 0, sizeof(remote_control_diag));
    pc_authorized = 0u;
    radio_sample_sequence = 0u; // 启动注册建立新的无线读取记录。
    source_selection_generation = 0u; // 上电后尚未选择过有效来源。
    gesture_switch_generation = 0u; // 上电后尚未收到有效的双开关组合。
    pc_sample_sequence = 0u; // 本机键鼠样本号只在模块初始化时归零。
    RevokePC(PlatformTimeNowMs(), REMOTE_PC_REVOKE_NONE);
    Daemon_Init_Config_s daemon_conf = {
        .transport_timeout_ms = REMOTE_RADIO_LEASE_MS,
        .data_timeout_ms = REMOTE_RADIO_LEASE_MS,
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
    rc_usart_instance = PlatformUartRegister(port, &conf);
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
    const uint32_t primask = PlatformIrqSaveAndDisable();
    const uint32_t check_time = PlatformTimeNowMs();
    const uint8_t online = (uint8_t)(rc_init_flag != 0u &&
        remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(check_time - remote_control_diag.last_valid_tick) <= REMOTE_RADIO_LEASE_MS &&
        DaemonIsOnline(rc_daemon_instance));
    remote_control_diag.online = online;
    PlatformIrqRestore(primask);
    return online;
}

void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot)
{
    if (snapshot == NULL) return;
    (void)RemoteControlIsOnline();
    uint32_t primask = PlatformIrqSaveAndDisable();
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t online = (uint8_t)(rc_init_flag != 0u &&
        remote_control_diag.has_valid_frame != 0u &&
        (uint32_t)(now - remote_control_diag.last_valid_tick) <= REMOTE_RADIO_LEASE_MS &&
        DaemonIsOnline(rc_daemon_instance));
    remote_control_diag.online = online;
    ExpirePC(now);
    snapshot->remote_online = (uint8_t)(online != 0u &&
        remote_control_diag.has_valid_frame != 0u && rc_init_flag != 0u);
    snapshot->input = rc_ctrl[TEMP];
    snapshot->pc_valid = (uint8_t)(snapshot->remote_online && pc_authorized);
    snapshot->radio_source_time_ms = remote_control_diag.last_valid_tick; // 继承接收事件时间，不使用本次查询时刻。
    snapshot->radio_sequence = radio_sample_sequence; // 与当前无线输入一起交付。
    snapshot->source_selection_generation = source_selection_generation; // 保留两次读取之间的来源变化。
    snapshot->gesture_switch_generation = gesture_switch_generation; // 和当前开关位置一起交付。
    snapshot->pc_source_time_ms = remote_control_diag.pc_last_advance_tick; // 键鼠使用自己的期限起点。
    snapshot->pc_sample_sequence = pc_sample_sequence; // 重复心跳和重复查询都保留已有编号。
    PlatformIrqRestore(primask);
}

uint8_t RemoteControlRecoveryStillValid(const RemoteControlSnapshot_s *snapshot, uint32_t now_ms)
{
    if (snapshot == NULL) return 0u; // 没有原始输入时不能验证恢复来源。
    const PlatformIrqState_t irq = PlatformIrqSaveAndDisable(); // 来源与代次必须一次检查。
    const uint16_t selected = snapshot->input.rc.switch_left; // 保存此前读取的人工来源档位。
    uint8_t valid = (uint8_t)(snapshot->remote_online && rc_init_flag && remote_control_diag.has_valid_frame &&
        snapshot->radio_sequence != 0u && snapshot->radio_sequence == radio_sample_sequence &&
        snapshot->source_selection_generation == source_selection_generation &&
        snapshot->radio_source_time_ms == remote_control_diag.last_valid_tick &&
        selected == rc_ctrl[TEMP].rc.switch_left &&
        (uint32_t)(now_ms - snapshot->radio_source_time_ms) <= REMOTE_RADIO_LEASE_MS); // 无线更新、切档或超时均要求重新采样。
    if (switch_is_up(selected))
        valid = (uint8_t)(valid && snapshot->pc_valid && pc_authorized &&
            snapshot->pc_sample_sequence != 0u && snapshot->pc_sample_sequence == pc_sample_sequence &&
            snapshot->pc_source_time_ms == remote_control_diag.pc_last_advance_tick &&
            (uint32_t)(now_ms - snapshot->pc_source_time_ms) <= REMOTE_PC_LEASE_MS); // 键鼠档还要保留独立心跳期限。
    else if (!switch_is_down(selected))
        valid = 0u; // 中间档不提供人工控制命令。
    PlatformIrqRestore(irq); // 恢复调用者原有中断状态。
    return valid; // 不触碰动作命令或运行许可。
}

RemoteMouseStatus_t RemoteControlConsumeMouse(uint32_t through_sequence, RemoteMouseBatch_t *batch)
{
    if (batch == NULL) return REMOTE_MOUSE_BAD_ARGUMENT; // 没有输出对象时不改变队列。
    RemoteMouseBatch_t result = {0}; // 默认交付空批次。
    if (through_sequence == 0u) { *batch = result; return REMOTE_MOUSE_EMPTY; } // 没有对应输入时不消费后来到达的数据。
    const uint32_t previous = PlatformIrqSaveAndDisable(); // 接收和消费共用同一队列保护。
    const uint32_t now = PlatformTimeNowMs(); // 本轮按当前本机时间检查事件期限。
    ExpirePC(now); // 消费者延迟时也要执行键鼠失联撤销。
    if (mouse_overflow != 0u)
    {
        ClearMousePending(); // 消费者已获知丢失，后续新事件重新开始。
        PlatformIrqRestore(previous); // 不在中断屏蔽时执行上层处理。
        *batch = result; // 溢出批次不交付部分位移。
        return REMOTE_MOUSE_OVERFLOW;
    }
    while (mouse_count != 0u)
    {
        const RemoteMouseEvent_t event = mouse_events[mouse_head]; // 只读取队首，维持接收顺序。
        if ((uint32_t)(through_sequence - event.sequence) > INT32_MAX) break; // 比快照更新的事件留到下一轮。
        mouse_head = (uint8_t)((mouse_head + 1u) % REMOTE_MOUSE_EVENT_CAPACITY); // 从队列移除当前事件。
        --mouse_count; // 消费和丢弃过期事件都释放一个槽。
        if ((uint32_t)(now - event.source_ms) > REMOTE_PC_LEASE_MS) continue; // 过期或将来事件不得执行。
        if (result.count == 0u ||
            ((uint32_t)(result.oldest_source_ms - event.source_ms) != 0u &&
             (uint32_t)(result.oldest_source_ms - event.source_ms) <= INT32_MAX))
            result.oldest_source_ms = event.source_ms; // 保存本批最早原始时间，允许时钟回绕。
        result.x += event.x; // 最多八个 int16 值相加，不会溢出 int32。
        result.y += event.y; // 同样累计竖直位移。
        ++result.count; // 只有未过期事件计入交付数量。
    }
    PlatformIrqRestore(previous); // 整批复制结束后恢复进入前的中断状态。
    *batch = result; // 一次交付结果，失败或空队列不会保留旧位移。
    return result.count != 0u ? REMOTE_MOUSE_OK : REMOTE_MOUSE_EMPTY;
}

uint8_t RemoteControlGetDiagnostics(RemoteControlDiagnostics_s *output)
{
    if (output == NULL)
        return 0u;

    RemoteControlDiagnostics_s candidate;
    const uint32_t primask = PlatformIrqSaveAndDisable();
    candidate = remote_control_diag;
    PlatformIrqRestore(primask);
    *output = candidate;
    return 1u;
}
