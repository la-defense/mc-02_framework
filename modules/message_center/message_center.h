/**
 * @file message_center.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 固定类型话题目录、快照和有界事件队列接口。
 * @note 启动时复制并校验目录；运行中不分配内存，句柄由模块长期保存。
 * @copyright Copyright (c) 2022
 */
#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stddef.h>
#include <stdint.h>
#include "message_freshness.h"

#define MESSAGE_CENTER_MAX_TOPICS 12u
#define MESSAGE_CENTER_MAX_SUBSCRIBERS 16u
#define MESSAGE_CENTER_MAX_PAYLOAD_BYTES 64u
#define MESSAGE_CENTER_MAX_QUEUE_DEPTH 4u
#define MESSAGE_CENTER_MAX_PAYLOAD_STORAGE_BYTES 2048u

typedef uint16_t MessageTopicId_t;
typedef uint8_t MessageEndpointId_t;

typedef enum
{
    MESSAGE_OK = 0,
    MESSAGE_ERR_ARGUMENT,
    MESSAGE_ERR_NOT_INITIALIZED,
    MESSAGE_ERR_DIRECTORY,
    MESSAGE_ERR_TOPIC,
    MESSAGE_ERR_RELATION,
    MESSAGE_ERR_TYPE,
    MESSAGE_ERR_EMPTY,
    MESSAGE_ERR_EXPIRED,
    MESSAGE_ERR_CLOCK_DOMAIN,
    MESSAGE_ERR_CAPACITY
} MessageStatus_t;

typedef enum
{
    MESSAGE_DELIVERY_SNAPSHOT = 1,
    MESSAGE_DELIVERY_EVENT_FIFO = 2
} MessageDelivery_t;

typedef enum
{
    MESSAGE_OVERFLOW_REPLACE_OLDEST = 1,
    MESSAGE_OVERFLOW_REJECT_NEWEST = 2
} MessageOverflow_t;

typedef enum
{
    MESSAGE_CLOCK_LOCAL = 1,
    MESSAGE_CLOCK_REMOTE_RAW = 2,
    MESSAGE_CLOCK_LOCAL_MAPPED = 3
} MessageClockDomain_t;

typedef struct
{
    MessageTopicId_t topic_id;
    uint16_t payload_size;
    uint32_t max_age_ms;
    uint8_t capacity;
    MessageDelivery_t delivery;
    MessageOverflow_t overflow;
    MessageClockDomain_t source_clock;
    uint32_t publisher_mask;
    uint32_t subscriber_mask;
} MessageTopicDescriptor_t;

typedef struct
{
    uint32_t published;
    uint32_t overwritten;
    uint32_t rejected;
    uint32_t expired;
} MessageTopicStats_t;

typedef struct MessagePublisher MessagePublisher_t;
typedef struct MessageSubscriber MessageSubscriber_t;

/** @brief 初始化固定话题目录；重复传入相同目录返回 MESSAGE_OK。
 * @param[in] descriptors 只读描述数组，调用期间至少可读count项；成功后内容被复制。
 * @param[in] count 目录项数，范围1～MESSAGE_CENTER_MAX_TOPICS。
 * @return 成功或完全相同的重复初始化返回 MESSAGE_OK；非法目录不改变现有目录。
 * @note 在任何注册接口前调用；每个payload不超过64字节，队列容量不超过4，期限为1～INT32_MAX毫秒。
 */
MessageStatus_t MessageCenterInit(const MessageTopicDescriptor_t *descriptors, size_t count);

/** @brief 按固定话题编号和端点登记发布者。
 * @param[in] topic_id 已在目录声明的话题编号。
 * @param[in] endpoint_id 声明的唯一发布端点，范围1～31。
 * @return 成功返回模块长期保存的发布句柄；失败返回NULL，不占用句柄。
 */
MessagePublisher_t *MessagePublisherRegister(MessageTopicId_t topic_id,
                                             MessageEndpointId_t endpoint_id);

/** @brief 按固定话题编号和端点登记订阅者。
 * @param[in] topic_id 已在目录声明的话题编号。
 * @param[in] endpoint_id 目录允许的订阅端点，范围1～31；同话题同端点只允许一次。
 * @return 成功返回模块长期保存的订阅句柄；句柄或总数据区不足时返回NULL。
 * @note 仅启动阶段注册；所有订阅者共用固定2048字节数据区，成功注册后不释放。
 */
MessageSubscriber_t *MessageSubscriberRegister(MessageTopicId_t topic_id,
                                               MessageEndpointId_t endpoint_id);

/** @brief 发布复制数据并保留源、接收、发布时间。
 * @param[in] publisher 有效发布句柄。
 * @param[in] data 至少可读目录声明的payload_size字节，返回后可立即复用。
 * @param[in] source_time_ms 来源产生时刻；远端原始时间只保存，不与本机时钟比较。
 * @param[in] received_time_ms 本地首次接收时刻，单位ms，0合法；转发时应保留原值。
 * @param[in] published_time_ms 本次写入话题的本地时刻，单位ms，不覆盖前两种时间。
 * @param[in] source_clock 必须与目录声明时钟域一致；跨设备映射由上层适配器完成。
 * @return 成功返回MESSAGE_OK；参数、话题或时钟域错误时不改变任何订阅者数据。
 */
MessageStatus_t MessagePublishAt(MessagePublisher_t *publisher, const void *data,
                                 uint32_t source_time_ms, uint32_t received_time_ms,
                                 uint32_t published_time_ms,
                                 MessageClockDomain_t source_clock);

/** @brief 发布本地任务刚生成的快照，三种本地时间取同一次HAL毫秒读取。
 * @param[in] publisher 有效发布句柄。
 * @param[in] data 至少可读目录声明的payload_size字节。
 * @return 成功返回MESSAGE_OK；其余失败不改变订阅者数据。
 */
MessageStatus_t MessagePublishNow(MessagePublisher_t *publisher, const void *data);

/** @brief 读取未过期的最新快照，不消费数据也不续期。
 * @param[in] subscriber 快照话题的订阅句柄。
 * @param[out] output 至少可写payload_size字节；失败时若长度正确则清零。
 * @param[in] output_size 必须等于目录声明长度。
 * @param[in] now_ms 本地毫秒时刻，0及计数回绕合法。
 * @param[out] freshness 可为NULL；成功时复制原始时间及序号，失败清零。
 * @return 成功返回MESSAGE_OK；空数据或超期分别返回MESSAGE_ERR_EMPTY/EXPIRED。
 */
MessageStatus_t MessageReadSnapshot(MessageSubscriber_t *subscriber, void *output,
                                    size_t output_size, uint32_t now_ms,
                                    MessageFreshness_t *freshness);

/** @brief 读取一条未过期事件并消费它；队列头部过期事件会先丢弃。
 * @param[in] subscriber 事件FIFO话题的订阅句柄。
 * @param[out] output 至少可写payload_size字节；失败时若长度正确则清零。
 * @param[in] output_size 必须等于目录声明长度。
 * @param[in] now_ms 本地毫秒时刻，0及计数回绕合法。
 * @param[out] freshness 可为NULL；成功时复制原始时间及序号，失败清零。
 * @return 成功返回MESSAGE_OK；无有效事件返回EMPTY或EXPIRED。
 */
MessageStatus_t MessageReadEvent(MessageSubscriber_t *subscriber, void *output,
                                 size_t output_size, uint32_t now_ms,
                                 MessageFreshness_t *freshness);

/** @brief 复制话题统计，不消费队列或推进过期处理。
 * @param[in] topic_id 已登记的话题编号。
 * @param[out] stats 输出统计；参数错误时保持原值。
 * @return 成功返回MESSAGE_OK；未初始化或未知编号返回对应错误。
 */
MessageStatus_t MessageGetTopicStats(MessageTopicId_t topic_id, MessageTopicStats_t *stats);

#endif
