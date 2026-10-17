/**
* @file rm_referee.h
* @brief 声明裁判系统接收数据、交互数据和串口服务接口。
* RefereeInit 返回的数据结构由接收回调持续更新；调用方不要释放它。
 */
#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

/** @brief 保存裁判系统接收字段、机器人编号和初始化状态。 */
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_referee_warning_t RefereeWarning;				   // 0x0104
	ext_dart_info_t DartInfo;							   // 0x0105
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207
	ext_projectile_allowance_t ProjectileAllowance;		   // 0x0208
	ext_rfid_status_t RfidStatus;						   // 0x0209
	ext_dart_client_cmd_t DartClientCmd;				   // 0x020A
	ext_ground_robot_position_t GroundRobotPosition;	   // 0x020B
	ext_radar_mark_data_t RadarMarkData;				   // 0x020C
	ext_sentry_info_t SentryInfo;						   // 0x020D
	ext_radar_info_t RadarInfo;							   // 0x020E
	ext_map_command_t MapCommand;						   // 0x0303

	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;

	uint8_t init_flag;

} referee_info_t;

/** @brief UI 用来记录哪些显示内容需要刷新；1表示对应数据发生变化。 */
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
	uint8_t refresh_flag : 2;
} Referee_Interactive_Flag_t;

/** @brief 保存 UI 和机器人间通信需要的应用状态及上次显示值。 */
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	chassis_mode_e chassis_mode;			 // 底盘模式
	gimbal_mode_e gimbal_mode;				 // 云台模式
	shoot_mode_e shoot_mode;				 // 发射模式设置
	friction_mode_e friction_mode;			 // 摩擦轮关闭
	lid_mode_e lid_mode;					 // 弹舱盖打开
	Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

	// 上一次的模式，用于flag判断
	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	Chassis_Power_Data_s Chassis_last_Power_Data;

	Bullet_Speed_e bullet_speed;
	float chassis_offset_angle;
	float pitch_angle;

} Referee_Interactive_info_t;

#pragma pack()

/**
* @brief 注册裁判系统串口和接收数据处理。
* @param referee_usart_handle 已初始化的 UART 句柄，不能为 NULL。
* @return 成功时返回持续更新的裁判数据地址；空句柄、重复注册或资源注册失败时返回 NULL。
* @note 接收数据在串口回调中解析；调用方不要释放返回地址。
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
* @brief 使用 DMA 发送裁判系统帧。
* @param send 可读发送缓冲区，发送期间保持内容不变。
* @param tx_len 发送字节数。
* @note 调用后会让当前任务等待115ms；不检查空指针，也不返回发送状态。
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

/** @brief 查询看门狗是否收到足够新的裁判串口数据；返回1在线，0离线。 */
uint8_t RefereeIsOnline(void);

/** @brief 返回最近一次串口接收回调的 HAL 毫秒计数；不代表该帧已通过校验。 */
uint32_t RefereeGetLastRxMs(void);

#endif // !REFEREE_H
