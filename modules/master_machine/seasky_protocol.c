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
#include "crc8.h"
#include "crc16.h"
#include "memory.h"

/*获取CRC8校验码*/
/** @brief 计算已有协议 CRC8，接口行为同 crc_8()。
 * @param[in] pchMessage 可读数据地址。
 * @param[in] dwLength 字节数。
 * @return CRC8 结果。
 */
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
/** @brief 计算已有协议 CRC16，接口行为同 crc_16()。
 * @param[in] pchMessage 可读数据地址。
 * @param[in] dwLength 字节数。
 * @return CRC16 结果。
 */
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
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

// 公开用法见头文件；内部静态变量共用，调用方须依次组包。
void get_protocol_send_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            float *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    data_len = float_length * 4 + 2;
    /*帧头部分*/
    tx_buf[0] = PROTOCOL_CMD_ID;
    tx_buf[1] = data_len & 0xff;        // 低位在前
    tx_buf[2] = (data_len >> 8) & 0xff; // 这里写长度的高字节。
    tx_buf[3] = crc_8(&tx_buf[0], 3);   // 获取CRC8校验位

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
    crc16 = crc_16(&tx_buf[0], data_len + 6);
    tx_buf[data_len + 6] = crc16 & 0xff;
    tx_buf[data_len + 7] = (crc16 >> 8) & 0xff;

    *tx_buf_len = data_len + 8;
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
