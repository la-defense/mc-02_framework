#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
static USARTInstance *instance;
static unsigned callbacks;
uint32_t HAL_GetTick(void) { return 0; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;(void)t;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
static void callback(void)
{
    const uint8_t *frame=USARTGetCurrentRxData(instance);
    CHECK(frame && frame!=instance->recv_buff && instance->last_recv_size==18);
    CHECK(frame[0]==0x25);
    memset(instance->recv_buff,0xff,18); /* DMA writes during parsing. */
    CHECK(frame[0]==0x25 && frame[17]==0x25);
    ++callbacks;
}
int main(void)
{
    UART_HandleTypeDef uart={0};
    USART_Init_Config_s config={.recv_buff_size=18,.usart_handle=&uart,.module_callback=callback};
    instance=USARTRegister(&config);
    CHECK(USARTGetCurrentRxData(instance)==NULL && USARTGetCurrentRxData(NULL)==NULL);
    memset(instance->recv_buff,0x25,18); HAL_UARTEx_RxEventCallback(&uart,18);
    memset(instance->recv_buff,0,18); USARTProcessRx();
    CHECK(callbacks==1 && USARTGetCurrentRxData(instance)==NULL);
    return 0;
}
