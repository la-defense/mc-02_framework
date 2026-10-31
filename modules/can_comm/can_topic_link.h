/** @file can_topic_link.h
 * @brief 为单条CAN链路注册接收派发，并服务一组话题收发。
 * @note 仅启动阶段初始化，运行期由同一个CAN派发任务串行服务。
 */
#ifndef CAN_TOPIC_LINK_H
#define CAN_TOPIC_LINK_H
#include "can_topic_bridge.h"
#include "can_time_sync.h"

typedef struct {
    CanTopicTransport_t transport;
    CanTopicBridge_t bridge;
    CanTimeSync_t *time_sync;
    uint32_t assembly_timeout_ms;
    CanTopicBridgeStatus_t last_receive;
    uint8_t ready;
} CanTopicLink_t;

/** @brief 校验话题和期限后注册CAN接收端点。
 * @param[out] link 调用前清零，成功后长期保存；失败保持原值。
 * @param[in] can_config CAN配置，仅在调用中读取；接收回调和id由本模块设置。
 * @param[in] outgoing,incoming 长期有效的订阅／发布句柄，至少一个非空。
 * @param[in] outgoing_codec,incoming_codec 启用方向的编码／解码配置，初始化时复制。
 * @param[in] map_time,map_context 远端时钟映射及长期上下文，缺失时远端数据保持失效。
 * @param[in] assembly_timeout_ms 拼接总期限，1～INT32_MAX毫秒。
 * @return 1为成功、0为参数／话题／资源错误；失败不占用CAN实例。
 * @note 调度器启动前串行调用，禁止CAN接收派发与初始化并发；成功后不提供运行期注销。
 */
uint8_t CanTopicLinkInit(CanTopicLink_t *link, const CAN_Init_Config_s *can_config,
    MessageSubscriber_t *outgoing, MessagePublisher_t *incoming,
    const CanTopicCodec_t *outgoing_codec, const CanTopicCodec_t *incoming_codec, CanTopicMapTime_t map_time,
    void *map_context, uint32_t assembly_timeout_ms);

/** @brief 将本链路接入双板对时，并撤销先前收到的消息。
 * @param[in,out] link 已初始化且带接收端点的链路，调度器启动前调用。
 * @param[in,out] sync 已初始化且绑定该链路CAN实例的对时对象，须长期有效。
 * @return 1为成功；参数、实例不匹配或撤销失败时返回0。
 */
uint8_t CanTopicLinkAttachTimeSync(CanTopicLink_t *link, CanTimeSync_t *sync);

/** @brief 检查来源同步并尝试提交一条新的本地数据。
 * @param[in,out] link 已初始化对象，由CAN接收派发所在任务串行调用。
 * @param[in] now_ms 当前本机毫秒时间。
 * @param[out] request_id 成功提交时写入，其他情况保持原值。
 * @param[out] sync_status 接收方向的同步结果；仅发送方向返回OK，参数失败保持原值。
 * @return 发送结果；仅接收方向返回同步结果。发送成功不代表接收同步有效。
 * @note 调用周期决定失同步撤销延迟；无映射时接收关闭，但不阻止发送本地新鲜数据。
 */
CanTopicBridgeStatus_t CanTopicLinkService(CanTopicLink_t *link, uint32_t now_ms,
                                          uint32_t *request_id, CanTopicBridgeStatus_t *sync_status);
#endif
