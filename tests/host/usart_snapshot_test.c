#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
static USARTInstance *instance;
static unsigned callbacks;
static HAL_StatusTypeDef receive_start_status = HAL_OK;
static unsigned receive_start_count;
static unsigned abort_receive_count;
#if defined(__linux__)
static long allocation_countdown = -1;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    return __real_calloc(count, size);
}
void __wrap_free(void *pointer) { __real_free(pointer); }
#endif
uint32_t HAL_GetTick(void) { return 0; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h;(void)p;(void)n;++receive_start_count;return receive_start_status; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;++abort_receive_count;return HAL_OK; }
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
    DMA_HandleTypeDef uart_dma={0};
    uart.hdmarx=&uart_dma;
    USART_Init_Config_s config={.recv_buff_size=18,.usart_handle=&uart,.module_callback=callback};

    CHECK(USARTRegister(NULL)==NULL);
    USART_Init_Config_s invalid=config;
    invalid.recv_buff_size=0;
    CHECK(USARTRegister(&invalid)==NULL);
    invalid=config;
    invalid.module_callback=NULL;
    CHECK(USARTRegister(&invalid)==NULL);
    invalid=config;
    invalid.usart_handle=NULL;
    CHECK(USARTRegister(&invalid)==NULL);
#if defined(__linux__)
    allocation_countdown=0;
    CHECK(USARTRegister(&config)==NULL);
    allocation_countdown=-1;
#endif
    receive_start_status=HAL_ERROR;
    CHECK(USARTRegister(&config)==NULL);
    CHECK(abort_receive_count==1u);
    receive_start_status=HAL_OK;
    instance=USARTRegister(&config);
    CHECK(instance!=NULL && receive_start_count==2u);
    CHECK(USARTRegister(&config)==NULL); /* Duplicate UART must not consume registry or DMA. */
    CHECK(USARTGetCurrentRxData(instance)==NULL && USARTGetCurrentRxData(NULL)==NULL);
    memset(instance->recv_buff,0x25,18); HAL_UARTEx_RxEventCallback(&uart,18);
    memset(instance->recv_buff,0,18); USARTProcessRx();
    CHECK(callbacks==1 && USARTGetCurrentRxData(instance)==NULL);

    UART_HandleTypeDef extra_uart[DEVICE_USART_CNT-1u]={0};
    DMA_HandleTypeDef extra_dma[DEVICE_USART_CNT-1u]={0};
    USART_Init_Config_s extra_config=config;
    extra_config.recv_buff_size=1u;
    for (unsigned i=0u; i<DEVICE_USART_CNT-1u; ++i)
    {
        extra_uart[i].hdmarx=&extra_dma[i];
        extra_config.usart_handle=&extra_uart[i];
        extra_config.module_callback=NULL;
        /* A callback is mandatory, so use the no-op test callback. */
        extra_config.module_callback=callback;
        CHECK(USARTRegister(&extra_config)!=NULL);
    }
    UART_HandleTypeDef overflow_uart={0};
    DMA_HandleTypeDef overflow_dma={0};
    overflow_uart.hdmarx=&overflow_dma;
    extra_config.usart_handle=&overflow_uart;
    CHECK(USARTRegister(&extra_config)==NULL);
    return 0;
}
