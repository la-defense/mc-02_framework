/**
* @file rm_referee.c
* @brief 接收裁判系统串口数据，检查帧后更新对应数据结构。
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
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static uint32_t referee_last_rx_ms = 0;
static referee_info_t referee_info;			  // 裁判系统数据

/**
* @brief 在一段串口数据中查找并解析完整的裁判系统帧。
* @param[in] buff 本次接收到的字节缓冲区。
* @param[in] recv_len 缓冲区内的字节数。
* @return 至少有一帧通过两次 CRC 检查时返回1，否则返回0。
 */
static uint8_t JudgeReadData(uint8_t *buff, uint16_t recv_len)
{
	uint16_t offset = 0; // 当前扫描位置
	uint8_t accepted = 0u;
	if (buff == NULL || recv_len < LEN_HEADER) // 空指针或不足一帧, 不作处理
		return 0u;

	// 在本次实际接收长度内滑动解析, 一个数据包可能包含多帧
	// 一段接收缓冲区可能含多帧；扫描位置会移到下一帧或向后找一个字节。
	while (offset + LEN_HEADER <= recv_len)
	{
		uint16_t frame_len;
		uint16_t data_len;
		uint32_t crc;

		// 判断帧头是否为 0xA5
		if (buff[offset + SOF] != REFEREE_SOF)
		{
			offset++; // 未命中帧头, 向后滑动继续查找
			continue;
		}

		// 帧头 CRC8 校验
		if (CRCCompute(CRC8_REFEREE, buff + offset, LEN_HEADER - 1u, &crc) != CRC_OK ||
            crc != buff[offset + LEN_HEADER - 1u])
		{
			offset++;
			continue;
		}

		// 2 字节小端 data_length(协议规定), 统计整帧长度
		data_len = (uint16_t)buff[offset + DATA_LENGTH] | ((uint16_t)buff[offset + DATA_LENGTH + 1] << 8);
		frame_len = LEN_HEADER + LEN_CMDID + data_len + LEN_TAIL;

		// 当前缓冲区不够一整帧时停止；剩余数据不从本次缓冲区外读取。
		if (frame_len < LEN_HEADER + LEN_CMDID + LEN_TAIL || frame_len > recv_len - offset)
			break;

		// 整包 CRC16 校验
		if (CRCCompute(CRC16_SP_REFEREE, buff + offset, frame_len - 2u, &crc) != CRC_OK ||
            crc != ((uint16_t)buff[offset + frame_len - 2u] |
                    ((uint16_t)buff[offset + frame_len - 1u] << 8u)))
		{
			offset++; // CRC 错误, 继续向后查找
			continue;
		}

		// 整帧合法后才允许刷新接收时间和发布字段。
		accepted = 1u;
		// 记录帧头, 拼接命令码。
		memcpy(&referee_info.FrameHeader, buff + offset, LEN_HEADER);
		referee_info.CmdID = (uint16_t)buff[offset + CMD_ID_Offset] | ((uint16_t)buff[offset + CMD_ID_Offset + 1] << 8);

		// 只处理已知命令且数据段达到本地结构体长度的帧。
		switch (referee_info.CmdID)
		{
		case ID_game_state: // 0x0001
			if (data_len >= LEN_game_state)
				memcpy(&referee_info.GameState, (buff + offset + DATA_Offset), LEN_game_state);
			break;
		case ID_game_result: // 0x0002
			if (data_len >= LEN_game_result)
				memcpy(&referee_info.GameResult, (buff + offset + DATA_Offset), LEN_game_result);
			break;
		case ID_game_robot_survivors: // 0x0003
			if (data_len >= LEN_game_robot_HP)
				memcpy(&referee_info.GameRobotHP, (buff + offset + DATA_Offset), LEN_game_robot_HP);
			break;
		case ID_event_data: // 0x0101
			if (data_len >= LEN_event_data)
				memcpy(&referee_info.EventData, (buff + offset + DATA_Offset), LEN_event_data);
			break;
		case ID_referee_warning: // 0x0104
			if (data_len >= LEN_referee_warning)
				memcpy(&referee_info.RefereeWarning, (buff + offset + DATA_Offset), LEN_referee_warning);
			break;
		case ID_dart_shoot_data: // 0x0105
			if (data_len >= LEN_dart_shoot_data)
				memcpy(&referee_info.DartInfo, (buff + offset + DATA_Offset), LEN_dart_shoot_data);
			break;
		case ID_game_robot_state: // 0x0201
			if (data_len >= LEN_game_robot_state)
				memcpy(&referee_info.GameRobotState, (buff + offset + DATA_Offset), LEN_game_robot_state);
			break;
		case ID_power_heat_data: // 0x0202
			if (data_len >= LEN_power_heat_data)
				memcpy(&referee_info.PowerHeatData, (buff + offset + DATA_Offset), LEN_power_heat_data);
			break;
		case ID_game_robot_pos: // 0x0203
			if (data_len >= LEN_game_robot_pos)
				memcpy(&referee_info.GameRobotPos, (buff + offset + DATA_Offset), LEN_game_robot_pos);
			break;
		case ID_buff_musk: // 0x0204
			if (data_len >= LEN_buff_musk)
				memcpy(&referee_info.BuffMusk, (buff + offset + DATA_Offset), LEN_buff_musk);
			break;
		case ID_robot_hurt: // 0x0206
			if (data_len >= LEN_robot_hurt)
				memcpy(&referee_info.RobotHurt, (buff + offset + DATA_Offset), LEN_robot_hurt);
			break;
		case ID_shoot_data: // 0x0207
			if (data_len >= LEN_shoot_data)
				memcpy(&referee_info.ShootData, (buff + offset + DATA_Offset), LEN_shoot_data);
			break;
		case ID_projectile_allowance: // 0x0208
			if (data_len >= LEN_projectile_allowance)
				memcpy(&referee_info.ProjectileAllowance, (buff + offset + DATA_Offset), LEN_projectile_allowance);
			break;
		case ID_rfid_status: // 0x0209
			if (data_len >= LEN_rfid_status)
				memcpy(&referee_info.RfidStatus, (buff + offset + DATA_Offset), LEN_rfid_status);
			break;
		case ID_dart_client_cmd: // 0x020A
			if (data_len >= LEN_dart_client_cmd)
				memcpy(&referee_info.DartClientCmd, (buff + offset + DATA_Offset), LEN_dart_client_cmd);
			break;
		case ID_ground_robot_position: // 0x020B
			if (data_len >= LEN_ground_robot_position)
				memcpy(&referee_info.GroundRobotPosition, (buff + offset + DATA_Offset), LEN_ground_robot_position);
			break;
		case ID_radar_mark_data: // 0x020C
			if (data_len >= LEN_radar_mark_data)
				memcpy(&referee_info.RadarMarkData, (buff + offset + DATA_Offset), LEN_radar_mark_data);
			break;
		case ID_sentry_info: // 0x020D
			if (data_len >= LEN_sentry_info)
				memcpy(&referee_info.SentryInfo, (buff + offset + DATA_Offset), LEN_sentry_info);
			break;
		case ID_radar_info: // 0x020E
			if (data_len >= LEN_radar_info)
				memcpy(&referee_info.RadarInfo, (buff + offset + DATA_Offset), LEN_radar_info);
			break;
		case ID_map_command: // 0x0303
			if (data_len >= LEN_map_command)
				memcpy(&referee_info.MapCommand, (buff + offset + DATA_Offset), LEN_map_command);
			break;
		case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
			if (data_len >= LEN_receive_data)
				memcpy(&referee_info.ReceiveData, (buff + offset + DATA_Offset), LEN_receive_data);
			break;
		}

		offset += frame_len; // 跳到下一帧
	}
	return accepted;
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	// CRC 错误或计算失败不能让坏数据维持在线状态。
	if (JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->last_recv_size))
	{
		DaemonReload(referee_daemon);
		referee_last_rx_ms = HAL_GetTick();
	}
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
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

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	osDelay(115);
}

uint8_t RefereeIsOnline(void)
{
	return (referee_daemon != NULL && DaemonIsOnline(referee_daemon) > 0) ? 1 : 0;
}

uint32_t RefereeGetLastRxMs(void)
{
	return referee_last_rx_ms;
}
