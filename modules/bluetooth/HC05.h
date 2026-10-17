/** @file HC05.h
 * @brief UART 蓝牙固定帧：0xAA、四字节内容、0x55；先初始化，再按固定长度收发。
 */
#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

#define  HC05_DATASIZE  4   // HC05接收和发送数据大小，根据需要修改

// HC05通信数据结构体，后续根据需要添加和修改
typedef struct 
{
    uint8_t  send_data[HC05_DATASIZE + 2];  // 发送数据，加上帧头和帧尾
    uint8_t  recv_data[HC05_DATASIZE];      // 接收数据
}HC05;

/** @brief 注册六字节 UART 接收并返回本模块的收发对象。
 * @param[in] hc05_usart_handle 已配置 UART 及接收 DMA 的句柄。
 * @return 内部静态对象地址，长期有效，不可释放；当前未检查 UART 注册失败，不能凭此地址判断成功。
 */
HC05 *HC05Init(UART_HandleTypeDef *hc05_usart_handle);

/** @brief 填写公共发送数组，并尝试以 IT 模式发送。
 * @param[in] data 至少可读 HC05_DATASIZE 字节，不能为 NULL。
 * @param[in] data_num 当前须等于 HC05_DATASIZE（4）；小于 4 时帧尾位置不随长度移动，大于 4 会越界。
 * @note 上次发送完成后才能再次调用，数组共用且接口不返回 HAL 结果。
 */
void HC05_SendData( uint8_t *data, uint8_t data_num);

#endif
