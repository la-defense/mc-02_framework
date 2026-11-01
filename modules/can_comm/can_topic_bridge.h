/** @file can_topic_bridge.h
 * @brief 将消息中心的一路最新数据与CAN话题传输相连。
 * @note 来源映射由调用者提供；无有效映射时禁止发布远端消息。
 */
#ifndef CAN_TOPIC_BRIDGE_H
#define CAN_TOPIC_BRIDGE_H
#include "can_topic_transport.h"
#include "message_center.h"

/** @brief 将远端毫秒时刻映射到本机，并给出毫秒误差上限；失败返回0。 */
typedef uint8_t (*CanTopicMapTime_t)(void *context, uint32_t remote_ms, uint32_t now_ms,
                                     uint32_t *local_ms, uint32_t *uncertainty_ms);
/** @brief 编码或解码固定长度数据；成功返回1，失败返回0。输入输出可能未对齐。 */
typedef uint8_t (*CanTopicConvert_t)(void *context, const void *input, size_t input_size,
                                     void *output, size_t output_size);
/** @brief 通知业务立即撤销依赖此输入的话题许可；回调必须有界且不得阻塞。 */
typedef void (*CanTopicInvalidateFn_t)(void *context);
typedef struct {
    CanTopicConvert_t convert; /**< 发送用编码，接收用解码；须完整检查输入。 */
    void *context; /**< 允许NULL；非NULL时须在桥接期间有效。 */
    uint8_t wire_size; /**< 线上内容字节数，1～64。 */
} CanTopicCodec_t;
typedef struct {
    CanTopicTransport_t *transport;
    MessageSubscriber_t *outgoing;
    MessagePublisher_t *incoming;
    MessageTopicDescriptor_t outgoing_desc, incoming_desc;
    CanTopicCodec_t outgoing_codec, incoming_codec;
    CanTopicMapTime_t map_time;
    void *map_context;
    CanTopicInvalidateFn_t invalidate_callback;
    void *invalidate_context;
    uint32_t last_sent_sequence;
    uint32_t pending_request_id;
    uint32_t last_remote_source_ms;
    uint32_t last_conservative_ms;
    uint8_t has_remote_source;
    uint8_t published_valid;
} CanTopicBridge_t;
typedef enum {
    CAN_BRIDGE_OK=0, CAN_BRIDGE_INCOMPLETE, CAN_BRIDGE_ARGUMENT, CAN_BRIDGE_CONFIG, CAN_BRIDGE_EMPTY,
    CAN_BRIDGE_EXPIRED, CAN_BRIDGE_UNSYNCED, CAN_BRIDGE_WIRE, CAN_BRIDGE_SEND, CAN_BRIDGE_PUBLISH
} CanTopicBridgeStatus_t;

/** @brief 绑定一个收发方向；允许只发送或只接收。
 * @param[out] bridge 长期保存的对象，失败保持原值。
 * @param[in] transport 已初始化传输对象。
 * @param[in] outgoing 本地最新数据订阅句柄，可为NULL；时间域须为本地或已映射。
 * @param[in] incoming 远端最新数据发布句柄，可为NULL；目录须使用LOCAL_MAPPED。
 * @param[in] outgoing_codec,incoming_codec 启用方向所用的固定长度编码／解码，初始化时复制。
 * @param[in] map_time,map_context 远端时钟映射和长期上下文；无映射时接收保持失效。
 * @return 状态；至少启用一个方向，目录长度为1～64字节。
 */
CanTopicBridgeStatus_t CanTopicBridgeInit(CanTopicBridge_t *bridge, CanTopicTransport_t *transport,
    MessageSubscriber_t *outgoing, MessagePublisher_t *incoming,
    const CanTopicCodec_t *outgoing_codec, const CanTopicCodec_t *incoming_codec,
    CanTopicMapTime_t map_time, void *map_context);

/** @brief 设置入站数据撤销回调；每次成功使已发布快照失效时调用。
 * @param[in,out] bridge 已初始化且有入站发布端的桥接对象。
 * @param[in] callback 可为NULL；调用须有界且不可阻塞。
 * @param[in] context 回调上下文，桥接期间须有效。
 * @return 参数和对象有效返回OK，否则返回ARGUMENT。
 * @note 在接收和服务任务启动前注册；快照撤销成功后，在触发撤销的任务上下文内调用。
 */
CanTopicBridgeStatus_t CanTopicBridgeSetInvalidateCallback(CanTopicBridge_t *bridge,
    CanTopicInvalidateFn_t callback, void *context);

/** @brief 读取未过期的本地数据并尝试提交一次，不重复发送已入队的同一版本。
 * @param[in,out] bridge 已初始化对象，调用者串行访问。
 * @param[in] now_ms 当前本机毫秒时刻。
 * @param[out] request_id 成功入队时填写；失败保持原值。
 * @return 状态；发送失败或排队未成功不消费版本，已入队的后续失败不自动重放。
 * @note 每个方向最多保留一个在途组；来源无效或出现新版本时取消尚未提交的旧帧。
 *       已全部交给HAL时须等待其结束，已经交给HAL的帧无法撤回。
 */
CanTopicBridgeStatus_t CanTopicBridgeSend(CanTopicBridge_t *bridge, uint32_t now_ms, uint32_t *request_id);

/** @brief 核对已拼完整消息并发布到配置的话题。
 * @param[in,out] bridge 已初始化对象，调用者串行访问。
 * @param[in] message 完整消息及本机首帧时间。
 * @param[in] now_ms 当前本机毫秒时刻。
 * @return 状态；失同步、过期、话题或长度不符时不发布。
 * @note 使用映射时刻减误差上限作为本地来源时间，避免误差使消息比源期限更长；
 *       同一来源时刻及更旧消息被拒绝。发送板复位后先撤销旧来源，再重新建立同步。
 */
CanTopicBridgeStatus_t CanTopicBridgePublish(CanTopicBridge_t *bridge, const CanTopicMessage_t *message,
                                             uint32_t now_ms);

/** @brief 消费传输对象中当前接收帧，完整后检查映射并发布。
 * @param[in,out] bridge 已初始化的桥接对象，由接收派发任务串行访问。
 * @param[in] now_ms 本机当前毫秒时刻。
 * @param[in] timeout_ms 单个消息的拼接期限，1～INT32_MAX毫秒。
 * @return 等待分片、发布成功或具体错误；未完整或校验失败不发布。
 * @note 调用者须确保transport对应当前回调的CAN对象；还须按周期调用CanTopicBridgeCheckSync。
 */
CanTopicBridgeStatus_t CanTopicBridgeReceive(CanTopicBridge_t *bridge, uint32_t now_ms, uint32_t timeout_ms);

/** @brief 撤销对端旧数据，并清除旧来源时刻和未完成的分片。
 * @param[in,out] bridge 已初始化且具有接收端点的桥接对象。
 * @return 成功、参数错误或撤销失败；撤销失败时不改变来源记录。
 * @note 仅在确认对端重启后调用；新消息仍须通过重新建立的时间映射。
 */
CanTopicBridgeStatus_t CanTopicBridgeResetPeer(CanTopicBridge_t *bridge);

/** @brief 检查已发布数据的映射是否仍有效；失同步时撤销旧话题和未完成分片。
 * @param[in,out] bridge 已初始化对象，由接收派发任务串行访问。
 * @param[in] now_ms 当前本机毫秒时刻。
 * @return 同步有效返回OK；失同步返回UNSYNCED，撤销失败返回PUBLISH。
 * @note 映射修正导致旧数据可能比原估计更早产生时也会撤销；调用周期决定发现延迟。
 *       此入口不自动重新建立映射或恢复控制许可。
 */
CanTopicBridgeStatus_t CanTopicBridgeCheckSync(CanTopicBridge_t *bridge, uint32_t now_ms);
#endif
