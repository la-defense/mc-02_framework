/**
 * @file message_center.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 按话题名连接发布者与订阅者，复制消息并记录发布时间。
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PUBSUB_H
#define PUBSUB_H

#include "stdint.h"
#include "message_freshness.h"

// 启动时先注册；同名话题长度必须相同。返回对象由模块保存，不可自行释放或改写链表。
#define MAX_TOPIC_NAME_LEN 32 // 最大的话题名长度,每个话题都有字符串来命名
#define MAX_TOPIC_COUNT 12    // 最多支持的话题数量
#define QUEUE_SIZE 1

typedef struct mqt
{
    /* 用数组模拟FIFO队列 */
    void *queue[QUEUE_SIZE];
    uint8_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t temp_size; // 当前队列长度
    MessageFreshness_t freshness;

    /* 指向下一个订阅了相同的话题的订阅者的指针 */
    struct mqt *next_subs_queue; // 使得发布者可以通过链表访问所有订阅了相同话题的订阅者
} Subscriber_t;

/**
 * @brief 发布者类型.每个发布者拥有发布者实例,并且可以通过链表访问所有订阅了自己发布的话题的订阅者
 *
 */
typedef struct ent
{
    /* 话题名称 */
    char topic_name[MAX_TOPIC_NAME_LEN + 1]; // 1个字节用于存放字符串结束符 '\0'
    uint8_t data_len;                        // 该话题的数据长度
    /* 指向第一个订阅了该话题的订阅者,通过链表访问所有订阅者 */
    Subscriber_t *first_subs;
    /* 指向下一个Publisher的指针 */
    struct ent *next_topic_node;
    uint8_t pub_registered_flag; // 用于标记该发布者是否已经注册
} Publisher_t;

/** @brief 注册订阅者，必要时先创建话题；每次成功都会新增一份订阅。
 * @param[in] name 非空话题名，须在前 32 个字节内含结尾 0，内容会复制保存。
 * @param[in] data_len 每条消息字节数，须为 1～255，通常传入 sizeof(消息类型)。
 * @return 成功返回长期有效对象；参数、同名长度、容量或内存分配不符时返回 NULL。
 */
Subscriber_t *SubRegister(char *name, uint8_t data_len);

/** @brief 注册发布者；已有同名且长度相同的话题时返回原对象。
 * @param[in] name 非空话题名，须在前 32 个字节内含结尾 0，内容会复制保存。
 * @param[in] data_len 消息字节数，须为 1～255，同名话题须一致。
 * @return 成功返回长期有效话题对象，参数、容量或分配失败时返回 NULL。
 */
Publisher_t *PubRegister(char *name, uint8_t data_len);

/** @brief 取出并消耗一条待取消息，不检查消息已过去多久。
 * @param[in,out] sub 有效订阅对象，成功后待取数量减 1。
 * @param[out] data_ptr 至少可写注册长度的区域。
 * @return 1 为已复制，0 为指针为空或没有待取消息；失败时输出不变。
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);

/** @brief 读取仍在时限内的最新消息，不消耗消息，也不重新计算发布时间。
 * @param[in] sub 订阅对象，NULL 或长度不符时失败。
 * @param[out] data_ptr 至少可写 data_len 字节，先清零；NULL 或 data_len 为 0 时失败。
 * @param[in] data_len 输出字节数，须等于注册长度。
 * @param[in] now_ms,max_age_ms 当前时间和允许过去的时长，单位 ms；等于时限允许。
 * @param[out] freshness 可为 NULL；先清零，成功时复制发布记录。
 * @return 1 为读取成功，0 为失败；可写的输出先清零，避免沿用旧命令。
 */
uint8_t SubGetSnapshot(Subscriber_t *sub, void *data_ptr, uint8_t data_len,
                       uint32_t now_ms, uint32_t max_age_ms,
                       MessageFreshness_t *freshness);

/** @brief 用当前 HAL 毫秒时间向所有订阅者复制消息，未取的旧消息被覆盖。
 * @param[in] pub 注册得到的发布对象。
 * @param[in] data_ptr 至少可读注册长度的区域；返回后可改写，不保留该地址。
 * @return 指针为空时为 0，其他情况为 1，包括没有订阅者；返回值不表示订阅者数量。
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);
/** @brief 按指定数据时间发布消息，其余行为同 PubPushMessage()。
 * @param[in] pub 发布对象。
 * @param[in] data_ptr 待复制的数据，长度由注册配置决定。
 * @param[in] timestamp_ms 数据原本产生时的毫秒时间，不能用派发时间掩盖排队耗时。
 * @return 指针为空时为 0，否则为 1；覆盖旧消息并增加各订阅者的发布序号。
 */
uint8_t PubPushMessageAt(Publisher_t *pub, void *data_ptr, uint32_t timestamp_ms);

#endif // !PUBSUB_H
