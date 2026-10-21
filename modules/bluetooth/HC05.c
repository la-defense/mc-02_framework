/** @file HC05.c
* @brief 处理六字节蓝牙帧；HC05Init成功后发送四字节内容，队列复制并在任务中通知。
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

static void HC05TxCompleted(void *context,const USARTTxEvent_t *event)
{
    HC05 *message=context;
    uint32_t saved=__get_PRIMASK();__disable_irq();message->last_tx=*event;
    if(event->result!=USART_TX_COMPLETE)++message->tx_failures;
    __set_PRIMASK(saved);
}
USARTSendStatus_t HC05_SendData(const uint8_t *data, uint8_t data_num)
{
    if(!hc05_init_flag)return USART_SEND_NOT_READY;
    if(data==NULL || data_num!=HC05_DATASIZE)return USART_SEND_BAD_ARGUMENT;
    uint8_t frame[HC05_BUFFERSIZE];frame[0]=FRAME_HEAD;
    for(unsigned i=0u;i<HC05_DATASIZE;++i)frame[i+1u]=data[i];
    frame[HC05_BUFFERSIZE-1u]=FRAME_END;
    USARTTxRequest_t request={frame,sizeof(frame),USART_TRANSFER_IT,50u,HC05TxCompleted,NULL,&hc05_msg};
    uint32_t id;return USARTSubmitTx(hc05_usart_instance,&request,&id);
}
USARTSendStatus_t HC05GetTxState(USARTTxEvent_t *event, uint32_t *failures)
{
    if (event == NULL || failures == NULL) return USART_SEND_BAD_ARGUMENT;
    if (!hc05_init_flag) return USART_SEND_NOT_READY;
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    *event = hc05_msg.last_tx; *failures = hc05_msg.tx_failures;
    __set_PRIMASK(saved); return USART_SEND_ACCEPTED;
}
