/**
 * @file can_comm.h
 * @author Neo neozng1@hnu.edu.cn
 * @brief  用于多机CAN通信的收发模块
 * @version 0.1
 * @date 2022-11-27
 *
 * @copyright Copyright (c) 2022 HNUYueLu EC all rights reserved
 *
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"
#include "message_freshness.h"

// 此模块把一份消息拆成多个经典 CAN 帧；接收收齐、检查帧尾和 CRC8 后才发布。
#define MX_CAN_COMM_COUNT 4 // 注意均衡负载,一条总线上不要挂载过多的外设

#define CAN_COMM_MAX_BUFFSIZE 60 // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'      // 帧头
#define CAN_COMM_TAIL 'e'        // 帧尾
#define CAN_COMM_OFFSET_BYTES 4  // 's'+ datalen + 'e' + crc8

/* CAN comm 结构体, 拥有CAN comm的app应该包含一个CAN comm指针 */
typedef struct
{
    CANInstance *can_ins;
    /* 发送部分 */
    uint8_t send_data_len; // 发送数据长度
    uint8_t send_buf_len;  // 发送缓冲区长度,为发送数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    /* 接收部分 */
    uint8_t recv_data_len; // 接收数据长度
    uint8_t recv_buf_len;  // 接收缓冲区长度,为接收数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];                  // 解包后的数据,调用CANCommGet()后cast成对应的类型通过指针读取即可
    /* 接收和更新标志位*/
    uint8_t recv_state;   // 接收状态,
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    uint8_t update_flag; // 收齐新消息后为 1，CANCommGet() 后清零。
    MessageFreshness_t freshness;
    uint32_t assembly_timestamp_ms; // 当前整包首帧接收时间，单位 ms，用于限制拼接耗时。

    DaemonInstance* comm_daemon;
} CANCommInstance;

/* CAN comm 初始化结构体 */
typedef struct
{
    CAN_Init_Config_s can_config; // CAN初始化结构体
    uint8_t send_data_len;        // 发送数据长度
    uint8_t recv_data_len;        // 接收数据长度

    uint32_t daemon_timeout_ms; // 合法完整消息的绝对期限，单位ms；0沿用1000ms默认值。
} CANComm_Init_Config_s;

/** @brief 注册 CAN 和离线回调，准备固定长度的消息收发对象。
* @param[in] comm_config 收发长度单位为字节，均不得超过 60；配置不能为 NULL，内部复制 CAN 配置。
* @return 成功返回长期有效实例，失败返回 NULL；失败会清理已分配对象，不创建控制任务。
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);

/** @brief 添加帧头、长度、CRC8 和帧尾，并原子提交完整分片组。
* @param[in] instance 初始化成功的实例，不能为 NULL；同一实例由调用者串行访问。
* @param[in] data 至少可读注册发送长度，不能为 NULL；队列复制完成后可立即复用。
* @return CAN队列提交状态；成功只表示完整分片组已入队，实际完成由请求状态独立记录。
* @note 队列容量不足时整包拒绝，不提交部分分片；HAL接受后的故障不重放旧分片。
 */
CAN_TxSubmitStatus_t CANCommSend(CANCommInstance *instance, const uint8_t *data);

/** @brief 清除更新标记并取得最后一次完整接收的内部数据，不检查时间。
* @param[in,out] instance 初始化成功的实例，不能为 NULL。
* @return 内部字节数组地址，下次收包可改写；不要释放，应用优先使用带时限的复制入口。
* @note 两端须使用相同字段顺序与字节排列，不可仅因结构体名称相同就直接解释数据。
 */
void *CANCommGet(CANCommInstance *instance);
/** @brief 复制仍在时限内的完整消息，不消耗或更新时间。
* @param[in] instance 收发实例，NULL 或接收长度不符时失败。
* @param[out] data 至少可写 data_len 字节，先清零；NULL 时失败。
* @param[in] data_len 输出字节数，须等于注册的接收长度且非 0。
* @param[in] now_ms,max_age_ms 当前时间和允许过去的毫秒数，等于时限也允许。
* @param[out] freshness 可选发布记录，先清零，成功时填写。
* @return 1 为成功，0 为失败；可写输出清零，防止应用继续使用旧值。
 */
uint8_t CANCommGetSnapshot(CANCommInstance *instance, void *data, uint8_t data_len,
                           uint32_t now_ms, uint32_t max_age_ms,
                           MessageFreshness_t *freshness);

/** @brief 检查完整消息时间和离线计时。
* @param[in] instance 收发实例，NULL 时返回 0。
* @return 1 为最近消息不超过 50 ms 且 Daemon 在线，否则为 0。
 */
uint8_t CANCommIsOnline(CANCommInstance *instance);

#endif // !CAN_COMM_H
