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
	USARTTxEvent_t uart_last_tx; // UART最近完成事件；USB下id为0，不进入SP线格式。
	float bullet_speed;
	uint16_t bullet_count;
} Vision_Status_t;

typedef enum
{
    VISION_TIME_SYNC_RESULT_NONE = 0,
    VISION_TIME_SYNC_RESULT_WAITING,
    VISION_TIME_SYNC_RESULT_SAMPLE_ACCEPTED,
    VISION_TIME_SYNC_RESULT_RESPONSE_REJECTED,
    VISION_TIME_SYNC_RESULT_TIMEOUT,
    VISION_TIME_SYNC_RESULT_SEND_REJECTED,
    VISION_TIME_SYNC_RESULT_RESET
} VisionTimeSyncLastResult_t;

typedef enum
{
    VISION_TIME_SYNC_OK = 0,
    VISION_TIME_SYNC_ERR_ARGUMENT,
    VISION_TIME_SYNC_ERR_NOT_INITIALIZED,
    VISION_TIME_SYNC_ERR_UNAVAILABLE,
    VISION_TIME_SYNC_ERR_NOT_SYNCHRONIZED,
    VISION_TIME_SYNC_ERR_EXPIRED,
    VISION_TIME_SYNC_ERR_TIME
} VisionTimeSyncStatus_t;

/** @brief 可复制读取的V2对时质量；偏差定义为远端时钟减本机时钟。 */
typedef struct
{
    int32_t remote_minus_local_ms;
    uint32_t sample_age_ms;
    uint32_t last_round_trip_ms;
    uint32_t request_id;
    uint32_t probes_accepted;
    uint32_t probe_timeouts;
    uint32_t rejected_replies;
    uint32_t transport_generation;
    VisionTimeSyncLastResult_t last_result;
    uint8_t synchronized;
    uint8_t sample_count;
    uint8_t request_pending;
} VisionTimeSyncSnapshot_t;

/** @brief 注册当前构建选定的视觉协议及传输，并启动链路离线计时。
* @param[in] _handle UART 配置时须提供有效句柄；USB VCP 配置时忽略，可以为 NULL。
* @return 成功返回兼容接收对象地址，长期有效且不可释放；任一注册失败返回NULL。
* @note 仅初始化阶段调用一次；控制任务用VisionGetSnapshot()复制读取。
* SP构建沿用固定帧解析；V2构建使用12字节头、精确负载长度和CRC16，
* 以组成帧首块的本机接收时刻记录来源接收时间，块中断、连接代号变化或组帧超时会清残帧。
* V2命令通过消息中心话题读取，返回对象只为保持初始化调用兼容。
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/** @brief 按当前协议周期发送状态；V2交替发送姿态和裁判摘要。
* @note 先初始化，由命令任务周期调用。SP状态由VisionUpdateTx()准备并按原帧格式发送；
*       V2以50ms节奏交替编码两类状态，字段各自保留源时刻和有效位。
*       USB使用单份在途复制缓冲，UART使用复制队列；忙时拒绝本次，不积压旧状态。
*       接口不返回结果，提交和传输计数由VisionGetStatus读取。
 */
void VisionSend(void);

/** @brief 检查时限后复制链路状态，与 VisionGetSnapshot(NULL, status) 相同。
* @param[out] status 调用方状态对象，NULL 时只做时限检查。
 */
void VisionGetStatus(Vision_Status_t *status);

/** @brief 在同一次读取中复制命令和链路状态，复制期间防止任务切换改写数据。
* @param[out] frame 可为 NULL；命令超过 50 ms 未更新时先清零，等于 50 ms 仍保留。
* @param[out] status 可为 NULL；online 还要求离线计时未到，last_rx_ms 使用原RxEvent通知时间。
* @note 单纯到时会保留上次接收时间，但 has_valid_frame 清零；离线回调另将时间清零。
*       USB连接代号失效时，即使尚未执行连接通知，也立即撤销旧命令。
 */
void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status);

/** @brief 复制V2对时状态；查询不发送探测、不推进样本或续期。
* @param[out] output 输出对象；失败保持原值。
* @return VISION_TIME_SYNC_OK；SP构建返回UNAVAILABLE，未初始化或空参数返回错误。
*/
VisionTimeSyncStatus_t VisionGetTimeSyncSnapshot(VisionTimeSyncSnapshot_t *output);

/** @brief 映射一个V2远端毫秒时刻到本机时基。
* @param[in] remote_ms V2帧携带的远端原始时刻，0合法。
* @param[in] now_local_ms 当前本机毫秒时刻，用于判断映射是否过期。
* @param[out] local_ms 映射结果；失败保持原值。
* @return VISION_TIME_SYNC_OK；未同步、过期、时间歧义或SP构建均有明确状态。
* @note 换算不改变消息接收时间，调用方仍须按原接收期限判断业务是否有效。
*/
VisionTimeSyncStatus_t VisionMapRemoteTime(uint32_t remote_ms,
                                           uint32_t now_local_ms,
                                           uint32_t *local_ms);

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
