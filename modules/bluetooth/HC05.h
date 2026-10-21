/** @file HC05.h
* @brief UART 蓝牙固定帧：0xAA、四字节内容、0x55；先初始化，再按固定长度收发。
*/
#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "bsp_usart.h"

#define  HC05_DATASIZE  4   // HC05接收和发送数据大小，根据需要修改

// HC05通信数据结构体，后续根据需要添加和修改
typedef struct 
{
    uint8_t  send_data[HC05_DATASIZE + 2];  // 保留旧成员；新发送入口使用局部帧，不借用此数组。
    USARTTxEvent_t last_tx;
    uint32_t tx_failures;
    uint8_t  recv_data[HC05_DATASIZE];      // 接收数据
}HC05;

/** @brief 注册六字节 UART 接收并返回本模块的收发对象。
* @param[in] hc05_usart_handle 已配置 UART 及接收 DMA 的句柄。
* @return 内部静态对象地址，长期有效，不可释放；注册失败返回NULL。
*/
HC05 *HC05Init(UART_HandleTypeDef *hc05_usart_handle);

/** @brief 将固定四字节内容复制为六字节帧并以IT方式入队。
 * @param[in] data 至少4个可读字节，成功后可立即复用。
 * @param[in] data_num 必须等于4，其他长度返回参数错误，不发送。
 * @return ACCEPTED仅表示入队；未初始化、队列满或后端错误明确返回。
 * @note 仅普通任务调用；总期限50ms，最终结果用HC05GetTxState读取；接收另行验证。
 */
USARTSendStatus_t HC05_SendData(const uint8_t *data, uint8_t data_num);
/** @brief 复制最近发送结果，不启动传输。
 * @param[out] event 非NULL，id为0表示尚未通知。
 * @param[out] failures 非NULL，失败通知累计次数。
 * @return ACCEPTED、未初始化时NOT_READY或BAD_ARGUMENT；失败保持输出。
 */
USARTSendStatus_t HC05GetTxState(USARTTxEvent_t *event, uint32_t *failures);
#endif
