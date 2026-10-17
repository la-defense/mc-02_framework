/** @file master_process.h
* @brief 视觉链路注册、命令读取与状态回传；先 VisionInit()，再周期更新和发送。
 */
#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "vision_protocol.h"
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

typedef struct
{
	uint8_t online;
	uint8_t has_valid_frame;
	uint8_t mode;
	uint32_t last_rx_ms;
	uint32_t rx_count;
	uint32_t tx_count;
	uint32_t tx_drop_count;
	uint32_t crc_error_count;
	float bullet_speed;
	uint16_t bullet_count;
} Vision_Status_t;

/** @brief 注册视觉通信及离线计时，清空收发帧；仅在初始化阶段调用一次。
* @param[in] _handle UART 配置时须提供有效句柄；USB VCP 配置时忽略，可以为 NULL。
* @return 成功返回模块内部接收对象地址，长期有效且不可释放；任一注册失败返回 NULL。
* @note 控制任务应使用 VisionGetSnapshot() 复制读取；失败时已申请的离线计时会撤销。
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/** @brief 检查旧命令是否到时，再给已准备的状态帧追加 CRC 并尝试发送。
* @note 先初始化；由命令任务周期调用，状态内容由 INS 调用 VisionUpdateTx() 更新。
*       不等待忙链路；USB 忙时记录丢弃次数，UART 忙时直接跳过；接口不返回发送结果。
 */
void VisionSend(void);

/** @brief 检查时限后复制链路状态，与 VisionGetSnapshot(NULL, status) 相同。
* @param[out] status 调用方状态对象，NULL 时只做时限检查。
 */
void VisionGetStatus(Vision_Status_t *status);

/** @brief 在同一次读取中复制命令和链路状态，复制期间防止任务切换改写数据。
* @param[out] frame 可为 NULL；命令超过 50 ms 未更新时先清零，等于 50 ms 仍保留。
* @param[out] status 可为 NULL；online 还要求离线计时未到，last_rx_ms 使用接收时的 HAL 毫秒值。
* @note 单纯到时会保留上次接收时间，但 has_valid_frame 清零；离线回调另将时间清零。
 */
void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status);

/** @brief 更新待回传的状态内容，本函数不发送、不检查数值范围。
* @param[in] mode 回传模式，当前调用处用 1 表示 READY，0 表示不允许自瞄。
* @param[in] q0,q1,q2,q3 姿态四元数，顺序为 w、x、y、z，无单位。
* @param[in] yaw,yaw_vel 左右角度和角速度，单位 rad、rad/s。
* @param[in] pitch,pitch_vel 上下角度和角速度，单位 rad、rad/s。
* @param[in] bullet_speed 弹速，单位 m/s。
* @param[in] bullet_count 累计发数。
 */
void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count);

#endif // !MASTER_PROCESS_H
