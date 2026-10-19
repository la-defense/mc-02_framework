/**
* @file rm_referee.c
* @brief 接收并检查裁判帧；RefereeSend由裁判任务串行复制到持久缓冲后发送。
* 只处理命令码已知且数据段长度足够的帧；未知命令会跳过。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static uint32_t referee_last_rx_ms = 0;
static referee_info_t referee_info;			  // 裁判系统数据

static uint8_t referee_has_valid;
typedef struct {
    uint8_t bytes[128];
    uint32_t time[128];
    size_t used, expected;
    uint32_t sequence;
    uint8_t have_sequence;
    volatile uint8_t reset_requested;
} RefereeStream;
static RefereeStream referee_stream;

// 只复制现有命令的既有前缀；完整协议字段核对留给MC02-0094。
static uint8_t RefereeAccept(const uint8_t *buff, size_t data_len, uint32_t received_ms)
{
    uint8_t accepted = 0u;
    uint16_t command = (uint16_t)buff[5] | ((uint16_t)buff[6] << 8u);
    switch (command)
    {
        case ID_game_state: // 0x0001
            if (data_len >= LEN_game_state) {
                memcpy(&referee_info.GameState, buff + DATA_Offset, LEN_game_state);
                accepted = 1u;
            }
            break;
        case ID_game_result: // 0x0002
            if (data_len >= LEN_game_result) {
                memcpy(&referee_info.GameResult, buff + DATA_Offset, LEN_game_result);
                accepted = 1u;
            }
            break;
        case ID_game_robot_survivors: // 0x0003
            if (data_len >= LEN_game_robot_HP) {
                memcpy(&referee_info.GameRobotHP, buff + DATA_Offset, LEN_game_robot_HP);
                accepted = 1u;
            }
            break;
        case ID_event_data: // 0x0101
            if (data_len >= LEN_event_data) {
                memcpy(&referee_info.EventData, buff + DATA_Offset, LEN_event_data);
                accepted = 1u;
            }
            break;
        case ID_referee_warning: // 0x0104
            if (data_len >= LEN_referee_warning) {
                memcpy(&referee_info.RefereeWarning, buff + DATA_Offset, LEN_referee_warning);
                accepted = 1u;
            }
            break;
        case ID_dart_shoot_data: // 0x0105
            if (data_len >= LEN_dart_shoot_data) {
                memcpy(&referee_info.DartInfo, buff + DATA_Offset, LEN_dart_shoot_data);
                accepted = 1u;
            }
            break;
        case ID_game_robot_state: // 0x0201
            if (data_len >= LEN_game_robot_state) {
                memcpy(&referee_info.GameRobotState, buff + DATA_Offset, LEN_game_robot_state);
                accepted = 1u;
            }
            break;
        case ID_power_heat_data: // 0x0202
            if (data_len >= LEN_power_heat_data) {
                memcpy(&referee_info.PowerHeatData, buff + DATA_Offset, LEN_power_heat_data);
                accepted = 1u;
            }
            break;
        case ID_game_robot_pos: // 0x0203
            if (data_len >= LEN_game_robot_pos) {
                memcpy(&referee_info.GameRobotPos, buff + DATA_Offset, LEN_game_robot_pos);
                accepted = 1u;
            }
            break;
        case ID_buff_musk: // 0x0204
            if (data_len >= LEN_buff_musk) {
                memcpy(&referee_info.BuffMusk, buff + DATA_Offset, LEN_buff_musk);
                accepted = 1u;
            }
            break;
        case ID_robot_hurt: // 0x0206
            if (data_len >= LEN_robot_hurt) {
                memcpy(&referee_info.RobotHurt, buff + DATA_Offset, LEN_robot_hurt);
                accepted = 1u;
            }
            break;
        case ID_shoot_data: // 0x0207
            if (data_len >= LEN_shoot_data) {
                memcpy(&referee_info.ShootData, buff + DATA_Offset, LEN_shoot_data);
                accepted = 1u;
            }
            break;
        case ID_projectile_allowance: // 0x0208
            if (data_len >= LEN_projectile_allowance) {
                memcpy(&referee_info.ProjectileAllowance, buff + DATA_Offset, LEN_projectile_allowance);
                accepted = 1u;
            }
            break;
        case ID_rfid_status: // 0x0209
            if (data_len >= LEN_rfid_status) {
                memcpy(&referee_info.RfidStatus, buff + DATA_Offset, LEN_rfid_status);
                accepted = 1u;
            }
            break;
        case ID_dart_client_cmd: // 0x020A
            if (data_len >= LEN_dart_client_cmd) {
                memcpy(&referee_info.DartClientCmd, buff + DATA_Offset, LEN_dart_client_cmd);
                accepted = 1u;
            }
            break;
        case ID_ground_robot_position: // 0x020B
            if (data_len >= LEN_ground_robot_position) {
                memcpy(&referee_info.GroundRobotPosition, buff + DATA_Offset, LEN_ground_robot_position);
                accepted = 1u;
            }
            break;
        case ID_radar_mark_data: // 0x020C
            if (data_len >= LEN_radar_mark_data) {
                memcpy(&referee_info.RadarMarkData, buff + DATA_Offset, LEN_radar_mark_data);
                accepted = 1u;
            }
            break;
        case ID_sentry_info: // 0x020D
            if (data_len >= LEN_sentry_info) {
                memcpy(&referee_info.SentryInfo, buff + DATA_Offset, LEN_sentry_info);
                accepted = 1u;
            }
            break;
        case ID_radar_info: // 0x020E
            if (data_len >= LEN_radar_info) {
                memcpy(&referee_info.RadarInfo, buff + DATA_Offset, LEN_radar_info);
                accepted = 1u;
            }
            break;
        case ID_map_command: // 0x0303
            if (data_len >= LEN_map_command) {
                memcpy(&referee_info.MapCommand, buff + DATA_Offset, LEN_map_command);
                accepted = 1u;
            }
            break;
        case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
            if (data_len >= LEN_receive_data) {
                memcpy(&referee_info.ReceiveData, buff + DATA_Offset, LEN_receive_data);
                accepted = 1u;
            }
            break;
    }

    if (accepted)
    {
        memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
        referee_info.CmdID = command;
        referee_last_rx_ms = received_ms;
        referee_has_valid = 1u;
        DaemonReload(referee_daemon);
    }
    return accepted;
}

static void RefereeShift(RefereeStream *stream, size_t count)
{
    stream->used -= count;
    memmove(stream->bytes, stream->bytes+count, stream->used);
    memmove(stream->time, stream->time+count, stream->used*sizeof(stream->time[0]));
    stream->expected = 0u;
}

// 逐字节保存数据及来源时间，CRC失败后移动后缀仍能找到它原来的接收块。
static void RefereeRxCallback(void *context, const USART_RxBlock_t *block)
{
    RefereeStream *stream = context;
    if (stream == NULL || block == NULL) return;
    taskENTER_CRITICAL();
    if (stream->reset_requested)
    {
        stream->used = stream->expected = 0u;
        stream->reset_requested = 0u;
    }
    taskEXIT_CRITICAL();
    uint32_t now = HAL_GetTick();
    if (!block->data || block->length == 0u || block->length > 255u)
    { stream->used = stream->expected = 0u; return; }
    if ((stream->have_sequence && block->sequence != stream->sequence+1u) ||
        (stream->used && (uint32_t)(now-stream->time[0]) > 50u))
        stream->used = stream->expected = 0u;
    stream->sequence = block->sequence;
    stream->have_sequence = 1u;
    if ((uint32_t)(now-block->received_ms) > 50u)
    { stream->used = stream->expected = 0u; return; }
    for (size_t i = 0; i < block->length; ++i)
    {
        // 离线任务只提出重建请求；残帧始终由本接收任务独占修改。
        if (stream->reset_requested) return;
        stream->bytes[stream->used] = block->data[i];
        stream->time[stream->used++] = block->received_ms;
        while (stream->used)
        {
            if (stream->bytes[0] != REFEREE_SOF) { RefereeShift(stream,1u); continue; }
            if (stream->used < LEN_HEADER) break;
            uint32_t crc;
            if (stream->expected == 0u)
            {
                if (CRCCompute(CRC8_REFEREE,stream->bytes,4u,&crc) != CRC_OK || crc != stream->bytes[4])
                { RefereeShift(stream,1u); continue; }
                size_t length = (size_t)stream->bytes[1] | ((size_t)stream->bytes[2] << 8u);
                // 当前接收范围只保留128字节候选，声明超大长度时立即继续找头。
                if (length > sizeof(stream->bytes)-9u)
                { RefereeShift(stream,1u); continue; }
                stream->expected = length+9u;
            }
            if (stream->used < stream->expected) break;
            size_t length = stream->expected;
            // 失败后只移走一个字节，后缀里的新帧头仍有机会被校验。
            if (CRCCompute(CRC16_SP_REFEREE,stream->bytes,length-2u,&crc) != CRC_OK ||
                crc != ((uint32_t)stream->bytes[length-2u] | ((uint32_t)stream->bytes[length-1u]<<8u)))
            { RefereeShift(stream,1u); continue; }
            taskENTER_CRITICAL();
            if (!stream->reset_requested && (uint32_t)(HAL_GetTick()-stream->time[0]) <= 50u)
                (void)RefereeAccept(stream->bytes,length-9u,stream->time[0]);
            taskEXIT_CRITICAL();
            RefereeShift(stream,length);
        }
    }
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
    (void)arg;
    taskENTER_CRITICAL();
    referee_stream.reset_requested = 1u;
    referee_has_valid = 0u;
    taskEXIT_CRITICAL();
	// 看门狗发现长时间没有接收后重新启动串口接收。
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    // 只允许注册一次，并要求调用方提供已初始化的 UART。
    if (referee_usart_handle == NULL || referee_usart_instance != NULL || referee_daemon != NULL)
        return NULL;

    Daemon_Init_Config_s daemon_conf = {
        .callback = RefereeLostCallback,
        .owner_id = &referee_info,
        .reload_count = 30,
    };
    referee_daemon = DaemonRegister(&daemon_conf);
    if (referee_daemon == NULL)
        return NULL;

    USART_Init_Config_s conf = {0};
    conf.module_callback = RefereeRxCallback;
    conf.context = &referee_stream;
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = RE_RX_BUFFER_SIZE;
    referee_usart_instance = USARTRegister(&conf);
    if (referee_usart_instance == NULL)
    {
        // 串口注册失败时撤销先前建立的 daemon，避免留下半套资源。
        (void)DaemonUnregister(referee_daemon);
        referee_daemon = NULL;
        return NULL;
    }
    return &referee_info;
}

RefereeSendStatus_t RefereeSend(const uint8_t *send, uint16_t tx_len)
{
    static uint8_t tx_buffer[128];
    if (send == NULL || tx_len == 0u || tx_len > sizeof(tx_buffer)) return REFEREE_SEND_BAD_ARGUMENT;
    if (referee_usart_instance == NULL) return REFEREE_SEND_NOT_INITIALIZED;
    // 单任务独占发送；必须先确认HAL结束借用旧缓冲，115ms等待不能替代此检查。
    if (!USARTIsReady(referee_usart_instance)) return REFEREE_SEND_BUSY;
    memcpy(tx_buffer, send, tx_len);
    USARTSendStatus_t status = USARTSendChecked(referee_usart_instance, tx_buffer, tx_len, USART_TRANSFER_DMA);
    switch (status)
    {
    case USART_SEND_ACCEPTED:
        osDelay(115);
        return REFEREE_SEND_ACCEPTED;
    case USART_SEND_BUSY: return REFEREE_SEND_BUSY;
    case USART_SEND_TIMEOUT: return REFEREE_SEND_TIMEOUT;
    default: return REFEREE_SEND_ERROR;
    }
}

uint8_t RefereeIsOnline(void)
{
	return (uint8_t)(referee_has_valid && (uint32_t)(HAL_GetTick()-referee_last_rx_ms) <= 300u &&
                     referee_daemon != NULL && DaemonIsOnline(referee_daemon));
}

uint32_t RefereeGetLastRxMs(void)
{
	return referee_last_rx_ms;
}
