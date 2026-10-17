/** @file can_comm.c
 * @brief 分片发送固定长度消息，接收时拼包并检查 CRC8，按真正收包时间判断是否仍可使用。
 */
#include "can_comm.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "memory.h"
#include "stdlib.h"
#include "crc8.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance *ins)
{
    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state = 0;   // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 */
static void CANCommRxCallback(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id;
    const uint32_t rx_timestamp_ms = _instance->rx_timestamp_ms;
    // 单帧排队已超过 50 ms 时放弃旧拼包；首帧到本帧超过 50 ms 也重新开始。
    if ((uint32_t)(HAL_GetTick() - rx_timestamp_ms) > MESSAGE_DEFAULT_MAX_AGE_MS)
    {
        CANCommResetRx(comm);
        return;
    }
    if (comm->recv_state != 0u &&
        (uint32_t)(rx_timestamp_ms - comm->assembly_timestamp_ms) > MESSAGE_DEFAULT_MAX_AGE_MS)
        CANCommResetRx(comm); // 本包没有及时收齐，丢掉已拼部分。

    /* 当前接收状态判断 */
    if (_instance->rx_buff[0] == CAN_COMM_HEADER && comm->recv_state == 0) // 之前尚未开始接收且此次包里第一个位置是帧头
    {
        if (_instance->rx_buff[1] == comm->recv_data_len) // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
        {
            comm->recv_state = 1;
            comm->assembly_timestamp_ms = rx_timestamp_ms; // 保留首帧时间，后续分片不能延长整包时限。
        }
        else
            return; // 直接跳过即可
    }

    if (comm->recv_state) // 已经收到过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
            { // 通过校验,复制数据到unpack_data中
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
                { // 数据量大的话考虑使用DMA
                    taskENTER_CRITICAL();
                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
                    comm->update_flag = 1;
                    comm->freshness.valid = 1u;
                    comm->freshness.timestamp_ms = rx_timestamp_ms;
                    comm->freshness.sequence++;
                    taskEXIT_CRITICAL();           // 数据更新flag置为1
                    DaemonReload(comm->comm_daemon); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
                }
            }
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }
    }
}

// 离线时只清未完成拼包；已发布消息由读取入口按时间拒绝。
static void CANCommLostCallback(void *cancomm)
{
    CANCommInstance *comm = (CANCommInstance *)cancomm;
    CANCommResetRx(comm);
    // 记录实际接收编号，便于区分是哪条链路离线。
    LOGWARNING("[can_comm] can comm rx[%lu] lost, reset rx state.",
               (unsigned long)comm->can_ins->rx_id);
}

CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    if (comm_config == NULL || comm_config->send_data_len > CAN_COMM_MAX_BUFFSIZE ||
        comm_config->recv_data_len > CAN_COMM_MAX_BUFFSIZE)
        return NULL;

    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(*ins));
    if (ins == NULL)
        return NULL;
    memset(ins, 0, sizeof(*ins));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len = (uint8_t)(comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES);
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len = (uint8_t)(comm_config->send_data_len + CAN_COMM_OFFSET_BYTES);
    ins->raw_sendbuf[0] = CAN_COMM_HEADER;
    ins->raw_sendbuf[1] = comm_config->send_data_len;
    ins->raw_sendbuf[ins->send_buf_len - 1u] = CAN_COMM_TAIL;

    Daemon_Init_Config_s daemon_config = {
        .callback = CANCommLostCallback,
        .owner_id = ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    if (ins->comm_daemon == NULL)
    {
        free(ins);
        return NULL;
    }

    CAN_Init_Config_s can_config = comm_config->can_config;
    can_config.id = ins;
    can_config.can_module_callback = CANCommRxCallback;
    ins->can_ins = CANRegister(&can_config);
    if (ins->can_ins == NULL)
    {
        (void)DaemonUnregister(ins->comm_daemon);
        free(ins);
        return NULL;
    }
    return ins;
}

void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    static uint8_t crc8;
    static uint8_t send_len;
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
    crc8 = crc_8(data, instance->send_data_len);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        CANTransmit(instance->can_ins, 1);
    }
}

void *CANCommGet(CANCommInstance *instance)
{
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

uint8_t CANCommGetSnapshot(CANCommInstance *instance, void *data, uint8_t data_len,
                           uint32_t now_ms, uint32_t max_age_ms,
                           MessageFreshness_t *freshness)
{
    if (freshness != NULL) memset(freshness, 0, sizeof(*freshness));
    if (data == NULL || data_len == 0u) return 0u;
    // 先清输出，实例、长度或时间检查失败时不保留上次控制数据。
    memset(data, 0, data_len);
    if (instance == NULL || instance->recv_data_len != data_len) return 0u;
    taskENTER_CRITICAL();
    if (MessageFreshnessIsValid(&instance->freshness, now_ms, max_age_ms))
    {
        memcpy(data, instance->unpacked_recv_data, data_len);
        if (freshness != NULL) *freshness = instance->freshness;
        taskEXIT_CRITICAL();
        return 1u;
    }
    taskEXIT_CRITICAL();
    return 0u;
}

uint8_t CANCommIsOnline(CANCommInstance *instance)
{
    if (instance == NULL) return 0u;
    MessageFreshness_t freshness;
    taskENTER_CRITICAL();
    freshness = instance->freshness;
    taskEXIT_CRITICAL();
    return (uint8_t)(MessageFreshnessIsValid(
        &freshness, HAL_GetTick(), MESSAGE_DEFAULT_MAX_AGE_MS) &&
        DaemonIsOnline(instance->comm_daemon));
}
