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
volatile RemoteControlDiagnostics_s remote_control_diag;

/* Decode into a private candidate. Rejected data must not touch edge history. */
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
    memcpy(c->key_count, rc_ctrl[TEMP].key_count, sizeof(c->key_count));
    for (unsigned i = 0; i < 16; ++i)
    {
        const uint16_t bit = (uint16_t)(1u << i);
        if (i == 4 || i == 5) continue;
        if ((c->key[KEY_PRESS].keys & bit) && !(rc_ctrl[LAST].key[KEY_PRESS].keys & bit) &&
            !(c->key[KEY_PRESS_WITH_CTRL].keys & bit) && !(c->key[KEY_PRESS_WITH_SHIFT].keys & bit))
            c->key_count[KEY_PRESS][i]++;
        for (unsigned kind = 1; kind < 3; ++kind)
            if ((c->key[kind].keys & bit) && !(rc_ctrl[LAST].key[kind].keys & bit))
                c->key_count[kind][i]++;
    }
    return 1u;
}

static void RemoteControlRxCallback(void)
{
    const uint8_t *frame = USARTGetCurrentRxData(rc_usart_instance);
    if (rc_usart_instance->last_recv_size != REMOTE_CONTROL_FRAME_SIZE || frame == NULL)
    {
        remote_control_diag.rejected_length++;
        return;
    }
    RC_ctrl_t candidate;
    if (!DecodeCandidate(frame, &candidate))
    {
        remote_control_diag.rejected_fields++;
        return;
    }
    rc_ctrl[TEMP] = candidate;
    rc_ctrl[LAST] = candidate;
    remote_control_diag.accepted_frames++;
    remote_control_diag.last_valid_tick = HAL_GetTick();
    remote_control_diag.has_valid_frame = 1u;
    DaemonReload(rc_daemon_instance);
    remote_control_diag.online = 1u;
    RobotSafetySetRcOnline(1u);
}

static void RCLostCallback(void *id)
{
    (void)id;
    remote_control_diag.offline_tick = HAL_GetTick();
    remote_control_diag.offline_last_valid_tick = remote_control_diag.last_valid_tick;
    remote_control_diag.offline_events++;
    remote_control_diag.has_valid_frame = remote_control_diag.online = 0u;
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    RobotSafetySetRcOnline(0u);
    EstopRequest(ESTOP_REASON_RC_OFFLINE);
    USARTServiceInit(rc_usart_instance);
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *handle)
{
    USART_Init_Config_s conf = {
        .module_callback = RemoteControlRxCallback,
        .usart_handle = handle,
        .recv_buff_size = REMOTE_CONTROL_FRAME_SIZE,
    };
    memset(rc_ctrl, 0, sizeof(rc_ctrl));
    memset((void *)&remote_control_diag, 0, sizeof(remote_control_diag));
    rc_usart_instance = USARTRegister(&conf);
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10,
        .callback = RCLostCallback,
        .owner_id = NULL,
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);
    RobotSafetySetRcOnline(0u);
    rc_init_flag = 1u;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline(void)
{
    const uint8_t online = (uint8_t)(rc_init_flag && remote_control_diag.has_valid_frame &&
                                   DaemonIsOnline(rc_daemon_instance));
    remote_control_diag.online = online;
    return online;
}
