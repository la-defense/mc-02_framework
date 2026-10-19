/** @file uart_stream_test.c
 * @brief 经真实UART队列、分帧和CRC检查裁判及SP跨块接收、恢复与来源时间。
 * @note 只替代HAL传输、任务与看门狗；不连接设备，Release仍执行全部检查。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#include "master_process.h"
#include "daemon.h"
#include "crc.h"
#include "test_check.h"

typedef struct {
    UART_HandleTypeDef handle;
    DMA_HandleTypeDef dma;
    uint8_t *rx;
    uint16_t capacity;
    USARTInstance *instance;
} Endpoint;
static Endpoint endpoints[2];
static uint32_t now_ms, reloads;
static unsigned daemon_count;
static DaemonInstance daemons[2];
static Daemon_Init_Config_s daemon_configs[2];
uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms) { (void)ms; }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle);
USARTInstance *__real_USARTRegister(USART_Init_Config_s *config);
USARTInstance *__wrap_USARTRegister(USART_Init_Config_s *config)
{
    USARTInstance *instance = __real_USARTRegister(config);
    for (unsigned i = 0; i < 2; ++i)
        if (config->usart_handle == &endpoints[i].handle) endpoints[i].instance = instance;
    return instance;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{
    for (unsigned i = 0; i < 2; ++i)
        if (h == &endpoints[i].handle) { endpoints[i].rx = p; endpoints[i].capacity = n; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *c)
{ TEST_CHECK(daemon_count < 2); daemon_configs[daemon_count] = *c; return &daemons[daemon_count++]; }
uint8_t DaemonUnregister(DaemonInstance *d) { (void)d; return 1u; }
void DaemonReload(DaemonInstance *d) { (void)d; ++reloads; }
uint8_t DaemonIsOnline(DaemonInstance *d) { (void)d; return 1u; }

// 逐位参考只用于生成测试字节；生产接收路径始终调用只读查表CRC。
static uint16_t reference(const uint8_t *p, size_t n, uint16_t polynomial, uint16_t initial)
{
    uint16_t c = initial;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0; b < 8; ++b) c = (c >> 1u) ^ ((c & 1u) ? polynomial : 0u);
    }
    return c;
}
static void sp_frame(uint8_t *p, uint8_t mode)
{
    Vision_Recv_s frame = {0};
    frame.head[0]='S'; frame.head[1]='P'; frame.mode=mode;
    memcpy(p,&frame,sizeof(frame));
    uint16_t crc=reference(p,sizeof(frame)-2u,0x8408,0xffff);
    p[sizeof(frame)-2u]=(uint8_t)crc; p[sizeof(frame)-1u]=(uint8_t)(crc>>8u);
}
static void enqueue(unsigned e, const uint8_t *p, size_t n)
{
    TEST_CHECK(n <= endpoints[e].capacity);
    memcpy(endpoints[e].rx,p,n);
    HAL_UARTEx_RxEventCallback(&endpoints[e].handle,(uint16_t)n);
    memset(endpoints[e].rx,0xcc,endpoints[e].capacity);
}
static void feed(unsigned e, const uint8_t *p, size_t n) { enqueue(e,p,n); USARTProcessRx(); }
static void restart(unsigned e)
{ TEST_CHECK(USARTServiceInit(endpoints[e].instance)); USARTProcessRx(); }
static void vision_cases(void)
{
    uint8_t frame[sizeof(Vision_Recv_s)], second[sizeof(Vision_Recv_s)], combined[255];
    Vision_Status_t status; Vision_Recv_s output;
    sp_frame(frame,1); sp_frame(second,2);
    TEST_CHECK(sizeof(frame)==29u);
    for (size_t split=1;split<sizeof(frame);++split) {
        restart(1); now_ms+=100; uint32_t first=now_ms, before=reloads;
        feed(1,frame,split); TEST_CHECK(reloads==before);
        ++now_ms; feed(1,frame+split,sizeof(frame)-split); VisionGetStatus(&status);
        TEST_CHECK(reloads==before+1 && status.last_rx_ms==first && status.has_valid_frame);
    }
    restart(1); now_ms+=100; uint32_t first=now_ms, before=reloads;
    for (size_t i=0;i<sizeof(frame);++i) { feed(1,frame+i,1); ++now_ms; }
    VisionGetStatus(&status); TEST_CHECK(reloads==before+1 && status.last_rx_ms==first);
    before=reloads; combined[0]=0x53; combined[1]=0;
    memcpy(combined+2,frame,sizeof(frame)); memcpy(combined+2+sizeof(frame),second,sizeof(frame));
    feed(1,combined,2+2*sizeof(frame)); VisionGetSnapshot(&output,&status);
    TEST_CHECK(reloads==before+2 && output.mode==2);
    uint32_t count=status.rx_count, received=status.last_rx_ms;
    frame[28]^=1; feed(1,frame,sizeof(frame)); VisionGetStatus(&status);
    TEST_CHECK(status.rx_count==count && status.last_rx_ms==received);
    frame[28]^=1;
    // 坏候选包含新帧头时，保留下来的后缀要保留其所在块的原时间。
    restart(1); now_ms+=100; first=now_ms;
    memset(combined,0,10); combined[0]='S'; combined[1]='P';
    memcpy(combined+10,frame,sizeof(frame)); before=reloads;
    feed(1,combined,29); now_ms+=5; feed(1,combined+29,10);
    VisionGetStatus(&status); TEST_CHECK(reloads==before+1 && status.last_rx_ms==first);
    const uint32_t starts[]={0u,UINT32_MAX-25u};
    for (unsigned i=0;i<2;++i) {
        restart(1); now_ms=starts[i]; feed(1,frame,10); first=now_ms;
        now_ms+=50; before=reloads; feed(1,frame+10,sizeof(frame)-10); VisionGetStatus(&status);
        TEST_CHECK(reloads==before+1 && status.last_rx_ms==first && status.has_valid_frame);
        ++now_ms; VisionGetStatus(&status); TEST_CHECK(!status.has_valid_frame && status.last_rx_ms==first);
        restart(1); now_ms=starts[i]; feed(1,frame,10); now_ms+=51;
        before=reloads; feed(1,frame+10,sizeof(frame)-10); TEST_CHECK(reloads==before);
    }
    restart(1); now_ms=5000; feed(1,frame,10);
    HAL_UART_ErrorCallback(&endpoints[1].handle); USARTProcessRx();
    before=reloads; feed(1,frame+10,sizeof(frame)-10); TEST_CHECK(reloads==before);
    feed(1,frame,sizeof(frame)); TEST_CHECK(reloads==before+1);
    restart(1); feed(1,frame,10);
    daemon_configs[0].callback(daemon_configs[0].owner_id);
    before=reloads; feed(1,frame+10,sizeof(frame)-10); TEST_CHECK(reloads==before);
    feed(1,frame,sizeof(frame)); TEST_CHECK(reloads==before+1);
    restart(1); const uint8_t noise=0;
    enqueue(1,&noise,1); enqueue(1,&noise,1); enqueue(1,frame,10); enqueue(1,&noise,1);
    before=reloads; USARTProcessRx(); feed(1,frame+10,sizeof(frame)-10); TEST_CHECK(reloads==before);
    endpoints[1].instance->rx_sequence=UINT32_MAX-1u;
    feed(1,frame,10); feed(1,frame+10,sizeof(frame)-10); TEST_CHECK(reloads==before+1);
    restart(1); now_ms=6000; enqueue(1,frame,sizeof(frame)); now_ms+=51;
    before=reloads; USARTProcessRx(); TEST_CHECK(reloads==before);
}
int main(void)
{
    for (unsigned i=0;i<2;++i) endpoints[i].handle.hdmarx=&endpoints[i].dma;
    TEST_CHECK(VisionInit(&endpoints[1].handle));
    vision_cases();
    return 0;
}
