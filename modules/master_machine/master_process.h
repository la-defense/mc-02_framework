#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include <stdint.h>

#define VISION_RECV_SIZE 64u
#define VISION_SEND_SIZE 64u
#define VISION_BULLET_SPEED_DEFAULT 30.0f // 弹速默认值(m/s), 后续接入裁判系统后改为实际限速

/* 以下枚举保留兼容(远程已移到 robot_def.h, 本地暂保留于此避免大范围改动) */
typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

#pragma pack(push, 1)

/* 上位机 -> 下位机，和 sp_vision_25 的 VisionToGimbal 对齐 */
typedef struct
{
	uint8_t head[2];   // 'S', 'P'
	uint8_t mode;      // 0: 不控制, 1: 控制云台不开火, 2: 控制云台并开火
	float yaw;
	float yaw_vel;
	float yaw_acc;
	float pitch;
	float pitch_vel;
	float pitch_acc;
	uint16_t crc16;
} Vision_Recv_s;

/* 下位机 -> 上位机，和 sp_vision_25 的 GimbalToVision 对齐 */
typedef struct
{
	uint8_t head[2];   // 'S', 'P'
	uint8_t mode;      // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
	float q[4];        // w, x, y, z
	float yaw;
	float yaw_vel;
	float pitch;
	float pitch_vel;
	float bullet_speed;
	uint16_t bullet_count;
	uint16_t crc16;
} Vision_Send_s;

typedef struct
{
	uint8_t online;
	uint8_t mode;
	uint32_t last_rx_ms;
	uint32_t rx_count;
	uint32_t tx_count;
	uint32_t tx_drop_count;
	uint32_t crc_error_count;
	float bullet_speed;
	uint16_t bullet_count;
} Vision_Status_t;

#pragma pack(pop)

/**
 * @brief 调用此函数初始化和视觉的通信(USB VCP 或 UART, 由 robot_def.h 宏切换)
 *
 * @param handle 用于和视觉通信的串口handle(仅 VISION_USE_UART 时使用)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend(void);

/**
 * @brief 获取视觉链路状态(在线/模式/收发计数/CRC错误)
 */
void VisionGetStatus(Vision_Status_t *status);

/**
 * @brief 每个控制周期把下位机状态塞给上位机
 *
 */
void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count);

#endif // !MASTER_PROCESS_H
