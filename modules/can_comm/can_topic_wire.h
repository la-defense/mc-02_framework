/** @file can_topic_wire.h
 * @brief 跨板话题的固定长度分片编码与有界拼接；调用者负责校验源时钟和话题权限。
 * @note 每个接收方向独占一个上下文，普通任务串行访问；本模块不读取时钟、不分配内存。
 */
#ifndef CAN_TOPIC_WIRE_H
#define CAN_TOPIC_WIRE_H

#include <stddef.h>
#include <stdint.h>

#define CAN_TOPIC_MAX_PAYLOAD 64u
#define CAN_TOPIC_MAX_FRAMES 13u
#define CAN_TOPIC_PACKET_MAX 73u

typedef enum {
    CAN_TOPIC_OK = 0, CAN_TOPIC_INCOMPLETE, CAN_TOPIC_ARGUMENT,
    CAN_TOPIC_FORMAT, CAN_TOPIC_ORDER, CAN_TOPIC_TIMEOUT, CAN_TOPIC_CRC
} CanTopicWireStatus_t;

typedef struct { uint8_t bytes[8]; uint8_t length; } CanTopicFrame_t;
typedef struct {
    uint16_t topic_id;
    uint8_t length;
    uint32_t source_ms;
    uint32_t first_rx_ms;
    uint8_t payload[CAN_TOPIC_MAX_PAYLOAD];
} CanTopicMessage_t;
typedef struct {
    uint8_t packet[CAN_TOPIC_PACKET_MAX];
    uint32_t first_rx_ms;
    uint8_t used, expected, sequence, active, last_sequence, has_completed;
} CanTopicReceiver_t;

/** @brief 将一条消息编码为最多13个经典 CAN 帧。
 * @param[in] message 话题编号非零、长度1～64；source_ms为来源时钟毫秒值。
 * @param[in] sequence 本消息线上序号，0～255；同一消息的所有分片使用同一值。
 * @param[out] frames 至少可写capacity项；失败保持原值。
 * @param[in] capacity frames项数，须足以容纳整组。
 * @param[out] count 成功时写帧数，失败保持原值。
 * @return 成功、参数错误或CRC计算失败。
 */
CanTopicWireStatus_t CanTopicEncode(const CanTopicMessage_t *message, uint8_t sequence,
                                     CanTopicFrame_t *frames, size_t capacity, size_t *count);

/** @brief 消费一个分片，完整且校验成功后复制消息。
 * @param[in,out] receiver 调用前清零，整个接收方向长期保存；错误会丢弃未完成拼接。
 * @param[in] frame 一帧可读数据，length为3～8字节。
 * @param[in] rx_ms 首次接收此帧的本地毫秒时刻，允许0和回绕。
 * @param[in] now_ms 本机当前毫秒时刻；排队或拼接超过timeout_ms时拒绝。
 * @param[in] timeout_ms 非零且不超过INT32_MAX；等于期限允许。
 * @param[out] output 完整成功才写入；其他结果保持原值。
 * @return 完整成功、等待后续分片或具体错误。
 * @note 输出的源时刻属于发送板，使用前必须经有效对时转换；本接口不保证数据新鲜。
 */
CanTopicWireStatus_t CanTopicReceive(CanTopicReceiver_t *receiver, const CanTopicFrame_t *frame,
                                     uint32_t rx_ms, uint32_t now_ms, uint32_t timeout_ms,
                                     CanTopicMessage_t *output);
#endif
