/** @file HC05.c
* @brief 处理固定六字节蓝牙帧，只有帧头、帧尾一致时才更新四字节内容。
*/
#include "HC05.h"
#include "bsp_usart.h"

#define HC05_BUFFERSIZE  HC05_DATASIZE+2  // HC05发送和接收数据buffer大小，不得大于256

#define FRAME_HEAD  0XAA    // 帧头
#define FRAME_END   0X55    // 帧尾

static HC05 hc05_msg;   // HC05通信数据
static USARTInstance *hc05_usart_instance;    // HC05串口通信实例

static uint8_t hc05_init_flag = 0;  // HC05初始化标志位

// *hc05_usart_instance串口回调函数
static void HC05RxCallback(void *context, const USART_RxBlock_t *block)
{
    HC05 *message = context;
    if (message == NULL || block->data == NULL || block->length != HC05_BUFFERSIZE) return;
    const uint8_t *rxbuff = block->data;
    if (rxbuff[0] == FRAME_HEAD && rxbuff[HC05_BUFFERSIZE - 1] == FRAME_END)
        for (int i = 0; i < HC05_DATASIZE; ++i) message->recv_data[i] = rxbuff[i + 1];
}

// HC05串口接收初始化
HC05 *HC05Init(UART_HandleTypeDef *hc05_usart_handle)
{
    USART_Init_Config_s conf = {0};
    conf.context = &hc05_msg;
    conf.module_callback = HC05RxCallback;
    conf.usart_handle = hc05_usart_handle;
    conf.recv_buff_size = HC05_BUFFERSIZE;
    hc05_usart_instance = USARTRegister(&conf);

    if (hc05_usart_instance == NULL) return NULL;
    hc05_init_flag = 1;
    return (HC05*)&hc05_msg;
}

// HC05串口发送函数，一次最多发送HC05_DATASIZE个数据
void HC05_SendData(uint8_t *data, uint8_t data_num)
{

    // 发送数据中加入帧头和帧尾
    hc05_msg.send_data[0] = FRAME_HEAD;

    // 内容放在帧头后；帧尾仍固定在数组末项，因此当前只支持完整的四字节内容。
    for (int i = 0; i < data_num; i++)
        hc05_msg.send_data[i+1] = data[i];
    
    hc05_msg.send_data[HC05_BUFFERSIZE - 1] = FRAME_END;

    // 发送数据
    USARTSend(hc05_usart_instance, hc05_msg.send_data, data_num+2, USART_TRANSFER_IT); 

}