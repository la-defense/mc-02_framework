/** @file usart_snapshot_test.c
 * @brief 用HAL模型检查UART接收独立副本、初始化回退和发送启动结果。
 * @note 主机运行，不连接串口；所有检查在Release下仍执行。
 */
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
static HAL_StatusTypeDef abort_status = HAL_OK;
static uint8_t start_interrupt;
static uint32_t now_ms;
static HAL_StatusTypeDef transmit_status = HAL_OK;
static unsigned transmit_calls;
#if defined(__linux__)
static long allocation_countdown = -1;
static unsigned allocation_attempts;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
void *__wrap_malloc(size_t size)
{
    ++allocation_attempts;
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    ++allocation_attempts;
    if (allocation_countdown == 0)
        return NULL;
    if (allocation_countdown > 0)
        --allocation_countdown;
    return __real_calloc(count, size);
}
void __wrap_free(void *pointer) { __real_free(pointer); }
#endif
uint32_t HAL_GetTick(void) { return now_ms; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{
    ++receive_start_count;
    if (start_interrupt) {
        start_interrupt=0u;
        memset(p,0x31,n);
        HAL_UARTEx_RxEventCallback(h,n);
    }
    return receive_start_status;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;++abort_receive_count;return abort_status; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;CHECK(t==0u || t==100u);++transmit_calls;return transmit_status; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
static USARTInstance *other;
static UART_HandleTypeDef *active_uart;
static uint32_t sequences[64], times[64];
static uint8_t values[64];
static int inject_arrival;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
static void callback(void *context, const USART_RxBlock_t *block)
{
    USARTInstance *owner = *(USARTInstance **)context;
    CHECK(owner && block->data && block->data != owner->recv_buff);
    CHECK(block->length == owner->recv_buff_size);
    uint8_t first = block->data[0], last = block->data[block->length - 1u];
    sequences[callbacks] = block->sequence;
    times[callbacks] = block->received_ms;
    values[callbacks++] = first;
    memset(owner->recv_buff, 0xff, owner->recv_buff_size);
    if (inject_arrival)
    {
        inject_arrival = 0;
        HAL_UARTEx_RxEventCallback(active_uart, owner->recv_buff_size);
        if (other) { memset(other->recv_buff, 0xee, other->recv_buff_size);
            HAL_UARTEx_RxEventCallback(other->usart_handle, other->recv_buff_size); }
        USARTProcessRx();
    }
    CHECK(block->data[0] == first && block->data[block->length - 1u] == last);
}
static void arrive(USARTInstance *owner, uint8_t value, uint16_t length)
{
    memset(owner->recv_buff, value, owner->recv_buff_size);
    HAL_UARTEx_RxEventCallback(owner->usart_handle, length);
}
int main(int argc, char **argv)
{
    UART_HandleTypeDef uart={0};
    USARTInstance sender={.usart_handle=&uart}, missing={0};
    uint8_t outgoing[1]={7};
    CHECK(USARTSendChecked(NULL,outgoing,1,USART_TRANSFER_DMA)==USART_SEND_BAD_ARGUMENT);
    CHECK(USARTSendChecked(&missing,outgoing,1,USART_TRANSFER_DMA)==USART_SEND_BAD_ARGUMENT);
    CHECK(USARTSendChecked(&sender,NULL,1,USART_TRANSFER_DMA)==USART_SEND_BAD_ARGUMENT);
    CHECK(USARTSendChecked(&sender,outgoing,0,USART_TRANSFER_DMA)==USART_SEND_BAD_ARGUMENT);
    CHECK(USARTSendChecked(&sender,outgoing,1,USART_TRANSFER_NONE)==USART_SEND_BAD_ARGUMENT);
    CHECK(USARTSendChecked(&sender,outgoing,1,(USART_TRANSFER_MODE)99)==USART_SEND_BAD_ARGUMENT);
    CHECK(transmit_calls==0);
    const HAL_StatusTypeDef results[]={HAL_OK,HAL_BUSY,HAL_ERROR,HAL_TIMEOUT};
    const USARTSendStatus_t expected[]={USART_SEND_ACCEPTED,USART_SEND_BUSY,USART_SEND_ERROR,USART_SEND_TIMEOUT};
    // 三种模式都经真实封装报告HAL结果，未知模式不再进入新接口的等待循环。
    for(unsigned mode=USART_TRANSFER_BLOCKING;mode<=USART_TRANSFER_DMA;++mode)
        for(unsigned result=0;result<4u;++result) {
            transmit_status=results[result];
            CHECK(USARTSendChecked(&sender,outgoing,1,(USART_TRANSFER_MODE)mode)==expected[result]);
        }
    CHECK(transmit_calls==12);
    transmit_status=HAL_OK;
    DMA_HandleTypeDef uart_dma={0};
    uart.hdmarx=&uart_dma;
    USART_Init_Config_s config={.recv_buff_size=18,.usart_handle=&uart,.module_callback=callback,.context=&instance};

    if (argc > 1 && strcmp(argv[1], "init-race") == 0)
    {
        // HAL启动尚未返回时就发生RxEvent，队列及上下文必须已经登记。
        start_interrupt=1u;
        instance=USARTRegister(&config); CHECK(instance);
        USARTProcessRx(); CHECK(callbacks==1u && values[0]==0x31u);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "quarantine") == 0)
    {
        // 启动及中止均失败后继续模拟中断，借用的内存仍须存在，但不能派发业务。
        receive_start_status=HAL_ERROR; abort_status=HAL_ERROR;
        CHECK(USARTRegister(&config)==NULL && abort_receive_count==1u);
        CHECK(USARTRegister(&config)==NULL);
        HAL_UARTEx_RxEventCallback(&uart,18); USARTProcessRx(); CHECK(callbacks==0u);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "slots") == 0)
    {
        UART_HandleTypeDef handles[DEVICE_USART_CNT+1u]={0};
        DMA_HandleTypeDef dmas[DEVICE_USART_CNT+1u]={0};
        for (unsigned i=0;i<=DEVICE_USART_CNT;++i) {
            handles[i].hdmarx=&dmas[i]; config.usart_handle=&handles[i]; config.recv_buff_size=1u;
            CHECK((USARTRegister(&config)!=NULL)==(i<DEVICE_USART_CNT));
        }
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "pool") == 0)
    {
        UART_HandleTypeDef handles[3] = {0}; DMA_HandleTypeDef dmas[3] = {0};
        for (unsigned i = 0; i < 3; ++i)
        {
            handles[i].hdmarx = &dmas[i]; config.usart_handle = &handles[i];
            config.recv_buff_size = 255u;
            CHECK((USARTRegister(&config) != NULL) == (i < 2u));
        }
        return 0;
    }
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
    active_uart = &uart;
    now_ms = 0u; arrive(instance,0x25,18); now_ms = 50u; USARTProcessRx();
    CHECK(callbacks == 1u && times[0] == 0u && values[0] == 0x25u);
    now_ms = 100u; arrive(instance,0x26,18); now_ms = 151u; USARTProcessRx();
    CHECK(callbacks == 1u);
    now_ms = 200u;
    arrive(instance,1,18); arrive(instance,2,18); arrive(instance,3,18); arrive(instance,4,18);
    USARTProcessRx(); CHECK(callbacks == 4u && values[1] == 1 && values[3] == 3);
    arrive(instance,5,18); USARTProcessRx(); CHECK(sequences[4] == sequences[3] + 2u);
    arrive(instance,6,19); arrive(instance,6,0); arrive(instance,7,18); USARTProcessRx();
    CHECK(callbacks == 6u && sequences[5] == sequences[4] + 3u);
    instance->rx_sequence = UINT32_MAX - 1u;
    arrive(instance,8,18); arrive(instance,9,18); USARTProcessRx();
    CHECK(sequences[6] == UINT32_MAX && sequences[7] == 0u);
    DMA_HandleTypeDef second_dma = {0}; UART_HandleTypeDef second_uart = {.hdmarx=&second_dma};
    config.usart_handle=&second_uart; config.context=&other;
    other=USARTRegister(&config); CHECK(other);
#if defined(__linux__)
    // 注册完成后禁止后续分配，接收、派发及恢复仍须正常运行。
    allocation_countdown=0;
    unsigned runtime_allocations=allocation_attempts;
#endif
    arrive(instance,10,18); inject_arrival=1; USARTProcessRx();
    CHECK(callbacks == 9u);
    USARTProcessRx(); CHECK(callbacks == 11u && values[9] == 0xffu && values[10] == 0xeeu);
    arrive(instance,11,18); HAL_UART_ErrorCallback(&uart); USARTProcessRx();
    CHECK(callbacks == 11u);
    arrive(instance,12,18); USARTProcessRx(); CHECK(callbacks == 12u);
    arrive(instance,13,18); CHECK(USARTServiceInit(instance)); USARTProcessRx();
    CHECK(callbacks == 12u);
    receive_start_status=HAL_ERROR;
    arrive(instance,14,18); USARTProcessRx(); CHECK(callbacks == 12u);
    unsigned starts=receive_start_count;
    USARTProcessRx(); CHECK(receive_start_count==starts);
    receive_start_status=HAL_OK;
    CHECK(USARTServiceInit(instance));
    USARTProcessRx();
    arrive(instance,15,18); USARTProcessRx(); CHECK(callbacks == 13u);
    now_ms = UINT32_MAX - 10u; arrive(instance,16,18); now_ms=39u; USARTProcessRx();
    CHECK(callbacks == 14u && values[13] == 16u);
#if defined(__linux__)
    CHECK(allocation_attempts==runtime_allocations);
#endif
    // 静态池容量在独立进程的pool用例检查，避免前面的注册占用影响结果。
    return 0;
}
