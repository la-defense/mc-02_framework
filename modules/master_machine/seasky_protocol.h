/** @file seasky_protocol.h
* @brief 保留的 A5 帧头串口协议，含标志位和 float 数据；与当前 SP 视觉帧分开使用。
* @note 接口没有接收长度或数组容量参数，调用方必须先确认一整帧已到达且输出空间足够。
*/
#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include "crc.h"

#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;


/** @brief 组装一帧并追加校验，不直接发送。
* @param[in] send_id 消息编号，按低字节在前写入。
* @param[in] flags_register 16 位标志值。
* @param[in] tx_data 至少可读 float_length 个 float，按本机 32 位 float 字节顺序复制；个数为0时允许 NULL。
* @param[in] float_length float 元素个数，不是字节数。
* @param[out] tx_buf 至少可写 4*float_length+10 字节。
* @param[out] tx_buf_len 最终帧字节数；失败置0，为 NULL 时直接返回错误。
* @return CRC_OK 或参数／CRC错误；失败时缓冲区可能已有部分内容，不得发送。
*/
CRCStatus_t get_protocol_send_data(uint16_t send_id,		 // 信号id
							uint16_t flags_register, // 16位寄存器
							float *tx_data,			 // 待发送的float数据
							uint8_t float_length,	 // float的数据长度
							uint8_t *tx_buf,		 // 待发送的数据帧
							uint16_t *tx_buf_len);	 // 待发送的数据帧长度

/** @brief 检查帧头 CRC8 和整帧 CRC16 后复制标志及数据。
* @param[in] rx_buf 完整且可读的帧，长度至少为帧头 data_length+8，data_length 须至少为 2。
* @param[out] flags_register 成功时写入标志值，失败保持原值。
* @param[out] rx_data 至少可写 data_length-2 字节，成功复制，失败保持原内容。
* @return 成功返回消息编号，失败返回 0；编号本身为 0 时无法单凭返回值区分。
* @note 地址不能为 NULL；未检查数组容量及 data_length 合理性，调用方须先检查，不要并发调用。
*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   uint16_t *flags_register, // 接收数据的16位寄存器地址
						   uint8_t *rx_data);			 // 接收的float数据存储地址

#endif
