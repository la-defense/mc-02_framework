/**
 * @file seasky_protocol.c
 * @author Liu Wei
 * @author modified by Neozng
* @brief 保留的 A5 串口帧组装与解析；当前 SP 视觉链路使用 vision_protocol.c。
 * @version 0.1
 * @date 2022-11-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "seasky_protocol.h"
#include "crc.h"
#include "memory.h"

// 旧 A5 帧保持原 CRC 参数；失败只拒绝本帧。
static uint8_t CRC8_Check_Sum(const uint8_t *data, size_t length)
{
    uint32_t crc;
    return data != NULL && length > 2u &&
           CRCCompute(CRC8_CAN_A5, data, length - 1u, &crc) == CRC_OK &&
           crc == data[length - 1u];
}

static uint8_t CRC16_Check_Sum(const uint8_t *data, size_t length)
{
    uint32_t crc;
    return data != NULL && length > 2u &&
           CRCCompute(CRC16_MODBUS_A5, data, length - 2u, &crc) == CRC_OK &&
           crc == ((uint16_t)data[length - 2u] | ((uint16_t)data[length - 1u] << 8u));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0];
        if (CRC8_Check_Sum(&rx_buf[0], 4))
        {
            pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
            pro->header.crc_check = rx_buf[3];
            pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
            return 1;
        }
    }
    return 0;
}

// 公开用法见头文件；失败时长度保持0，调用方不得发送未完成内容。
CRCStatus_t get_protocol_send_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            float *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    uint32_t crc16;
    uint32_t crc8;
    uint16_t data_len;
    if (tx_buf_len == NULL) return CRC_ERROR_ARGUMENT;
    *tx_buf_len = 0u;
    // 零长度数据允许空地址，其余实际使用的地址必须存在。
    if (tx_buf == NULL || (tx_data == NULL && float_length != 0u))
        return CRC_ERROR_ARGUMENT;

    data_len = float_length * 4 + 2;
    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;        // 低位在前
    tx_buf[2] = (data_len >> 8) & 0xff; // 这里写长度的高字节。
    CRCStatus_t status = CRCCompute(CRC8_CAN_A5, tx_buf, 3u, &crc8);
    if (status != CRC_OK) return status;
    tx_buf[3] = (uint8_t)crc8;

    /*数据的信号id*/
    tx_buf[4] = send_id & 0xff;
    tx_buf[5] = (send_id >> 8) & 0xff;

    /*建立16位寄存器*/
    tx_buf[6] = flags_register & 0xff;
    tx_buf[7] = (flags_register >> 8) & 0xff;

    /*float数据段*/
    // 每个 float 复制四个原始字节，不在此处改变数值或字节顺序。
    for (int i = 0; i < 4 * float_length; i++)
    {
        tx_buf[i + 8] = ((uint8_t *)(&tx_data[i / 4]))[i % 4];
    }

    /*整包校验*/
    status = CRCCompute(CRC16_MODBUS_A5, tx_buf, data_len + 6u, &crc16);
    if (status != CRC_OK) return status;
    tx_buf[data_len + 6] = crc16 & 0xff;
    tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 8;
    return CRC_OK;
}
// 先检查帧头，再按帧头声明的长度检查整帧；两次都通过才改写调用方输出。
uint16_t get_protocol_info(uint8_t *rx_buf,          // 接收到的原始数据
                           uint16_t *flags_register, // 接收数据的16位寄存器地址
                           uint8_t *rx_data)         // 接收的float数据存储地址
{
    // 放在静态区,避免反复申请栈上空间
    static protocol_rm_struct pro;
    static uint16_t date_length;

    if (protocol_heade_Check(&pro, rx_buf))
    {
        date_length = OFFSET_BYTE + pro.header.data_length;
        if (CRC16_Check_Sum(&rx_buf[0], date_length))
        {
            *flags_register = (rx_buf[7] << 8) | rx_buf[6];
            memcpy(rx_data, rx_buf + 8, pro.header.data_length - 2);
            return pro.cmd_id;
        }
    }
    return 0;
}
