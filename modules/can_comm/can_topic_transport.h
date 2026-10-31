/** @file can_topic_transport.h
 * @brief 将跨板话题帧交给已有CAN批量队列，并从接收帧拼接消息。
 * @note 此层不注册CAN、不发布话题；调用者先注册资源并在普通任务串行使用。
 */
#ifndef CAN_TOPIC_TRANSPORT_H
#define CAN_TOPIC_TRANSPORT_H
#include "bsp_can.h"
#include "can_topic_wire.h"

typedef struct {
    CANInstance *can; /**< 已注册的长期有效CAN对象。 */
    CanTopicReceiver_t receiver; /**< 仅由本实例的接收调用修改。 */
    uint8_t next_sequence; /**< 下次成功入队使用的序号。 */
} CanTopicTransport_t;

/** @brief 初始化调用者保存的传输状态。
 * @param[out] transport 长期保存的对象，失败保持原值。
 * @param[in] can 已注册的CAN对象，句柄及发送编号在生命周期内保持有效。
 * @return 1表示成功，0表示参数错误；须在收发之前调用，不得对运行中对象重复初始化。
 */
uint8_t CanTopicTransportInit(CanTopicTransport_t *transport, CANInstance *can);

/** @brief 复制完整分片组到CAN队列。
 * @param[in,out] transport 已初始化对象；仅成功入队时推进序号。
 * @param[in] message 内容及发送板源时刻，调用期间可读；入队成功后可以复用。
 * @param[in] deadline_ms 从入队开始的期限，范围1～INT32_MAX毫秒。
 * @param[out] request_id CAN队列编号；失败保持原值。
 * @return CAN队列提交状态，成功仅表示整组被复制。
 * @note 调用者须先确认源消息未过期，并限制队列期限；完成结果由CAN请求状态查询。
 *       更高优先级请求可使本组失败并放弃余帧，已交给HAL的帧不能撤回。
 */
CAN_TxSubmitStatus_t CanTopicTransportSend(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t deadline_ms, uint32_t *request_id);

/** @brief 提交话题帧组，保留来源数据的本机绝对截止时刻。
 * @param[in,out] transport 已初始化的传输对象。
 * @param[in] message 待编码话题，成功提交时复制。
 * @param[in] expiry_ms 本机HAL毫秒截止时刻；允许回绕。
 * @param[out] request_id 成功时写入，失败保持原值。
 * @return 队列提交结果；已到期时拒绝。
 */
CAN_TxSubmitStatus_t CanTopicTransportSendUntil(CanTopicTransport_t *transport,
    const CanTopicMessage_t *message, uint32_t expiry_ms, uint32_t *request_id);

/** @brief 消费CAN接收对象当前帧，完成后复制结果。
 * @param[in,out] transport 已初始化对象，接收回调中普通任务串行调用。
 * @param[in] now_ms 当前本机毫秒时刻。
 * @param[in] timeout_ms 组包期限，1～INT32_MAX毫秒。
 * @param[out] output 成功后复制，其他结果保持原值。
 * @return 拼包状态；成功不表示远端源时间已经映射或数据可以控制执行器。
 */
CanTopicWireStatus_t CanTopicTransportReceive(CanTopicTransport_t *transport,
    uint32_t now_ms, uint32_t timeout_ms, CanTopicMessage_t *output);
#endif
