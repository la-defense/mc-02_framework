/** @file vision_protocol_v2.h
 * @brief V2视觉业务帧及瞄准、导航、姿态和裁判摘要的字节接口。
 * @note 帧头12字节、负载最多112字节、末尾CRC16 2字节；内存结构体不能直接上线路。
 */
#ifndef VISION_PROTOCOL_V2_H
#define VISION_PROTOCOL_V2_H

#include <stddef.h>
#include <stdint.h>

#define VISION_V2_VERSION 2u
#define VISION_V2_HEADER_SIZE 12u
#define VISION_V2_CRC_SIZE 2u
#define VISION_V2_MAX_PAYLOAD 112u
#define VISION_V2_MAX_FRAME (VISION_V2_HEADER_SIZE + VISION_V2_MAX_PAYLOAD + VISION_V2_CRC_SIZE)
#define VISION_V2_STREAM_BLOCK_MAX 255u
#define VISION_V2_ASSEMBLY_TIMEOUT_MS 50u
#define VISION_V2_AIM_MAX_AGE_MS 50u
#define VISION_V2_NAV_MAX_AGE_MS 100u

typedef enum
{
    VISION_V2_AIM_COMMAND = 0x01,
    VISION_V2_NAVIGATION_COMMAND = 0x02,
    VISION_V2_POSTURE_STATUS = 0x81,
    VISION_V2_REFEREE_SUMMARY = 0x82
} VisionV2MessageType_t;

typedef enum
{
    VISION_V2_OK = 0,
    VISION_V2_ERR_ARGUMENT,
    VISION_V2_ERR_LENGTH,
    VISION_V2_ERR_VERSION,
    VISION_V2_ERR_TYPE,
    VISION_V2_ERR_CRC,
    VISION_V2_ERR_VALUE
} VisionV2Status_t;

typedef struct
{
    uint8_t mode; /* 0=无目标，1=使用瞄准数据；协议不包含开火请求。 */
    float yaw_rad;
    float yaw_rate_rad_s;
    float yaw_accel_rad_s2;
    float pitch_rad;
    float pitch_rate_rad_s;
    float pitch_accel_rad_s2;
} VisionV2AimCommand_t;

enum
{
    VISION_V2_NAV_TRANSLATION_VALID = 1u << 0,
    VISION_V2_NAV_YAW_RATE_VALID = 1u << 1
};

typedef struct
{
    uint8_t valid_flags;
    float vx_m_s;          /* 机器人本体坐标，x向前，单位m/s。 */
    float vy_m_s;          /* 机器人本体坐标，y向左，单位m/s。 */
    float yaw_rate_rad_s;  /* 逆时针为正，单位rad/s。 */
} VisionV2NavigationCommand_t;

enum
{
    VISION_V2_POSTURE_ORIENTATION_VALID = 1u << 0,
    VISION_V2_POSTURE_RATE_VALID = 1u << 1
};

typedef struct
{
    uint8_t valid_flags;
    uint8_t robot_state; /* 0=未就绪，1=READY；不发送C枚举内存。 */
    float quaternion_wxyz[4];
    float yaw_rad;
    float yaw_rate_rad_s;
    float pitch_rad;
    float pitch_rate_rad_s;
} VisionV2PostureStatus_t;

enum
{
    VISION_V2_REF_ROBOT_STATE_VALID = 1u << 0,
    VISION_V2_REF_POWER_HEAT_VALID = 1u << 1,
    VISION_V2_REF_SHOOT_RECORD_VALID = 1u << 2
};

typedef struct
{
    uint8_t valid_flags;
    uint8_t robot_id;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t chassis_power_limit_w;
    float bullet_speed_limit_m_s;
    uint8_t power_output_flags;
    uint16_t buffer_energy_j;
    uint16_t heat_17mm;
    uint16_t heat_42mm;
    uint8_t last_shot_frequency_hz;
    float last_shot_speed_m_s;
    uint32_t robot_state_source_ms;
    uint32_t power_heat_source_ms;
    uint32_t shoot_record_source_ms;
} VisionV2RefereeSummary_t;

typedef struct
{
    uint8_t type;
    uint16_t sequence;
    uint32_t source_time_ms; /* 远端时钟原值；未完成对时前不可与本机比较。 */
    uint16_t payload_length;
    uint8_t payload[VISION_V2_MAX_PAYLOAD];
    uint32_t received_time_ms;
    uint32_t generation;
} VisionV2Frame_t;

typedef struct
{
    uint8_t bytes[VISION_V2_MAX_FRAME];
    uint32_t byte_time_ms[VISION_V2_MAX_FRAME];
    size_t used;
    uint32_t previous_block_sequence;
    uint32_t generation;
    uint8_t have_previous_block;
} VisionV2Stream_t;

typedef void (*VisionV2FrameCallback_t)(void *context, const VisionV2Frame_t *frame);

/** @brief 返回V2消息类型固定的负载长度。
 * @param[in] type V2支持的消息编号。
 * @param[out] length 输出精确字节数；失败保持原值。
 * @return VISION_V2_OK或未知类型／空参数错误。
 */
VisionV2Status_t VisionV2GetPayloadSize(uint8_t type, size_t *length);

/** @brief 编码完整V2帧并追加CRC16_MODBUS_A5。
 * @note 字节顺序为：`MC`、版本、类型、负载长度LE16、序号LE16、远端源时刻LE32、负载、CRC LE16。
 * @param[in] type 固定消息编号，payload必须具有该类型的精确长度。
 * @param[in] sequence 16位线上序号，允许回绕。
 * @param[in] source_time_ms 来源时钟原值，不作本地时钟换算。
 * @param[in] payload 可读payload_length字节。
 * @param[in] payload_length 必须匹配type规定长度。
 * @param[out] output 至少可写VISION_V2_MAX_FRAME字节；失败不修改输出。
 * @param[out] output_length 成功时写总帧长；失败保持原值。
 * @return VISION_V2_OK或参数、长度、类型、CRC错误。
 */
VisionV2Status_t VisionV2EncodeFrame(uint8_t type, uint16_t sequence,
                                    uint32_t source_time_ms, const uint8_t *payload,
                                    size_t payload_length, uint8_t *output,
                                    size_t *output_length);

/** @brief 解码瞄准payload；六个浮点以小端IEEE-754 binary32保存。
 * @param[in] payload 恰好25字节。
 * @param[out] command 输出候选；非法模式或非有限值时保持原值。
 * @return VISION_V2_OK或参数、长度、数值错误。
 */
VisionV2Status_t VisionV2DecodeAim(const uint8_t *payload, size_t length,
                                   VisionV2AimCommand_t *command);

/** @brief 解码导航payload；速度范围为±2m/s，角速度范围为±10rad/s。
 * @param[in] payload 恰好13字节。
 * @param[out] command 输出候选；非法标志或数值时保持原值。
 * @return VISION_V2_OK或参数、长度、数值错误。
 */
VisionV2Status_t VisionV2DecodeNavigation(const uint8_t *payload, size_t length,
                                          VisionV2NavigationCommand_t *command);

/** @brief 按固定字段顺序编码34字节姿态状态负载。
 * @param[in] status 已检查的姿态候选，浮点必须有限。
 * @param[out] payload 至少34字节；失败保持原值。
 * @return VISION_V2_OK或参数／数值错误。
 */
VisionV2Status_t VisionV2EncodePosture(const VisionV2PostureStatus_t *status,
                                       uint8_t *payload, size_t capacity);

/** @brief 编码36字节裁判摘要负载，保留各组成组独立的来源时刻。
 * @param[in] summary 已按有效位检查的裁判摘要。
 * @param[out] payload 至少36字节；失败保持原值。
 * @return VISION_V2_OK或参数／数值错误。
 */
VisionV2Status_t VisionV2EncodeRefereeSummary(const VisionV2RefereeSummary_t *summary,
                                              uint8_t *payload, size_t capacity);

/** @brief 清空流解析器的残帧和接收块连续性。
 * @param[in,out] stream 调用方长期保存并串行使用的对象。
 */
void VisionV2StreamReset(VisionV2Stream_t *stream);

/** @brief 处理一个接收块并按序交付块内及跨块完整帧。
 * @param[in,out] stream 已清零或重置的长期解析状态，单任务独占。
 * @param[in] data 至少可读length字节，回调结束后不再借用。
 * @param[in] length 范围1～255字节。
 * @param[in] received_ms 该块首次接收的本机毫秒时刻，0合法。
 * @param[in] block_sequence 接收块序号；缺块时清除残帧，回绕允许。
 * @param[in] generation USB连接代号；变化时清除残帧。
 * @param[in] now_ms 与received_ms同一时基；组帧超过50ms即清除。
 * @param[in] complete 完整合法帧的同步回调，不能为空。
 * @param[in] context 原样传给回调，可为空。
 * @param[out] frame_errors 可为空；合法调用时写格式或CRC错误总数。
 * @return VISION_V2_OK或输入／指针错误；失败不改解析状态。
 */
VisionV2Status_t VisionV2StreamFeed(VisionV2Stream_t *stream, const uint8_t *data,
                                   size_t length, uint32_t received_ms,
                                   uint32_t block_sequence, uint32_t generation,
                                   uint32_t now_ms, VisionV2FrameCallback_t complete,
                                   void *context, uint32_t *frame_errors);

#endif
