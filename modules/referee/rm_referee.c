/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff, uint16_t recv_len)
{
	uint16_t offset = 0; // 当前扫描位置
	if (buff == NULL || recv_len < LEN_HEADER) // 空指针或不足一帧, 不作处理
		return;

	// 在本次实际接收长度内滑动解析, 一个数据包可能包含多帧
	while (offset + LEN_HEADER <= recv_len)
	{
		uint16_t frame_len;
		uint16_t data_len;

		// 判断帧头是否为 0xA5
		if (buff[offset + SOF] != REFEREE_SOF)
		{
			offset++; // 未命中帧头, 向后滑动继续查找
			continue;
		}

		// 帧头 CRC8 校验
		if (Verify_CRC8_Check_Sum(buff + offset, LEN_HEADER) != TRUE)
		{
			offset++;
			continue;
		}

		// 2 字节小端 data_length(协议规定), 统计整帧长度
		data_len = (uint16_t)buff[offset + DATA_LENGTH] | ((uint16_t)buff[offset + DATA_LENGTH + 1] << 8);
		frame_len = LEN_HEADER + LEN_CMDID + data_len + LEN_TAIL;

		// 边界校验: 帧长非法或未完整落在本次接收缓冲内则停止解析
		if (frame_len < LEN_HEADER + LEN_CMDID + LEN_TAIL || frame_len > recv_len - offset)
			break;

		// 整包 CRC16 校验
		if (Verify_CRC16_Check_Sum(buff + offset, frame_len) != TRUE)
		{
			offset++; // CRC 错误, 继续向后查找
			continue;
		}

		// 整帧合法: 记录帧头, 拼接命令码
		memcpy(&referee_info.FrameHeader, buff + offset, LEN_HEADER);
		referee_info.CmdID = (uint16_t)buff[offset + CMD_ID_Offset] | ((uint16_t)buff[offset + CMD_ID_Offset + 1] << 8);

		// 数据从第 8 个字节开始(offset + DATA_Offset), 拷贝前核对数据段长度防止越界
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
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->last_recv_size);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

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
