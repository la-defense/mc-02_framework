/** @file uart_stream_test.c
 * @brief 经真实UART队列、分帧和CRC检查裁判及SP跨块接收、恢复与来源时间。
 * @note 只替代HAL传输、任务与看门狗；不连接设备，Release仍执行全部检查。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#include "master_process.h"
#include "rm_referee.h"
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
static size_t referee_frame(uint8_t *p, uint16_t command, size_t payload, uint8_t value)
{
    memset(p, value, payload + 9u);
    p[0] = 0xa5; p[1] = (uint8_t)payload; p[2] = (uint8_t)(payload >> 8u); p[3] = 0;
    p[4] = (uint8_t)reference(p,4,0x8c,0xff);
    p[5] = (uint8_t)command; p[6] = (uint8_t)(command >> 8u);
    uint16_t crc = reference(p,payload+7u,0x8408,0xffff);
    p[payload+7u] = (uint8_t)crc; p[payload+8u] = (uint8_t)(crc >> 8u);
    return payload+9u;
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
static void referee_cases(referee_info_t *info)
{
    uint8_t frame[255], bad[255], combined[255];
    size_t n=referee_frame(frame,ID_game_result,1,5);
    // 固定字节另外用反射后binascii.crc_hqx核对CRC16，避免只有自编自解。
    const uint8_t fixed[]={0xa5,1,0,0,0x68,2,0,5,0x03,0x97};
    TEST_CHECK(memcmp(frame,fixed,n)==0);
    TEST_CHECK(!RefereeIsOnline());
    const struct { uint16_t command; size_t length; } supported[] = {
        {ID_game_state,LEN_game_state}, {ID_game_result,LEN_game_result},
        {ID_game_robot_survivors,LEN_game_robot_HP}, {ID_event_data,LEN_event_data},
        {ID_referee_warning,LEN_referee_warning}, {ID_dart_shoot_data,LEN_dart_shoot_data},
        {ID_game_robot_state,LEN_game_robot_state}, {ID_power_heat_data,LEN_power_heat_data},
        {ID_game_robot_pos,LEN_game_robot_pos}, {ID_buff_musk,LEN_buff_musk},
        {ID_robot_hurt,LEN_robot_hurt}, {ID_shoot_data,LEN_shoot_data},
        {ID_projectile_allowance,LEN_projectile_allowance}, {ID_rfid_status,LEN_rfid_status},
        {ID_dart_client_cmd,LEN_dart_client_cmd}, {ID_ground_robot_position,LEN_ground_robot_position},
        {ID_radar_mark_data,LEN_radar_mark_data}, {ID_sentry_info,LEN_sentry_info},
        {ID_radar_info,LEN_radar_info}, {ID_map_command,LEN_map_command},
        {ID_student_interactive,LEN_receive_data}
    };
    // 既有命令的最短前缀、少一字节及额外后缀，分别锁定原来的兼容规则。
    for (size_t i=0;i<sizeof(supported)/sizeof(supported[0]);++i) {
        size_t length=referee_frame(bad,supported[i].command,supported[i].length,0);
        uint32_t previous=reloads; feed(0,bad,length);
        TEST_CHECK(reloads==previous+1 && info->CmdID==supported[i].command);
        referee_info_t previous_info=*info;
        length=referee_frame(bad,supported[i].command,supported[i].length-1u,0);
        feed(0,bad,length);
        TEST_CHECK(reloads==previous+1 && memcmp(&previous_info,info,sizeof(*info))==0);
        length=referee_frame(bad,supported[i].command,supported[i].length+1u,0);
        feed(0,bad,length); TEST_CHECK(reloads==previous+2);
    }
    for (size_t split=1; split<n; ++split) {
        restart(0); now_ms+=100;
        uint32_t before=reloads, first=now_ms;
        feed(0,frame,split); TEST_CHECK(reloads==before);
        now_ms+=1; feed(0,frame+split,n-split);
        TEST_CHECK(reloads==before+1 && RefereeGetLastRxMs()==first && info->CmdID==ID_game_result);
    }
    restart(0); now_ms+=100; uint32_t first=now_ms, before=reloads;
    for (size_t i=0;i<n;++i) { feed(0,frame+i,1); ++now_ms; }
    TEST_CHECK(reloads==before+1 && RefereeGetLastRxMs()==first);
    memcpy(combined,frame,n); memcpy(combined+n,frame,n); feed(0,combined,2*n);
    TEST_CHECK(reloads==before+3);
    referee_info_t saved=*info; before=reloads;
    referee_frame(bad,0xffff,1,4); feed(0,bad,n);
    referee_frame(bad,ID_power_heat_data,0,4); feed(0,bad,9);
    memcpy(bad,frame,n); bad[n-1]^=1; feed(0,bad,n);
    memcpy(bad,frame,n); bad[4]^=1; feed(0,bad,n);
    TEST_CHECK(reloads==before && memcmp(&saved,info,sizeof(saved))==0);
    // 声明超大长度的合法帧头立即被拒绝，后面的正确帧仍可找到。
    memcpy(combined,frame,5); combined[1]=0xff; combined[2]=0xff;
    combined[4]=(uint8_t)reference(combined,4,0x8c,0xff);
    memcpy(combined+5,frame,n); feed(0,combined,5+n); TEST_CHECK(reloads==before+1);
    // 旧候选CRC失败后保留的后缀，必须继续使用原块时间。
    restart(0); now_ms+=100; first=now_ms;
    referee_frame(bad,0xffff,20,0); memcpy(bad+7,frame,n);
    feed(0,bad,7+n); now_ms+=5; feed(0,bad+7+n,29-7-n);
    TEST_CHECK(RefereeGetLastRxMs()==first);
    // 当前支持的最大127字节交互帧，逐分割位置执行真实生产解码。
    n=referee_frame(frame,ID_student_interactive,118,0);
    for (size_t split=1;split<n;++split) {
        restart(0); now_ms+=100; first=now_ms; before=reloads;
        feed(0,frame,split); ++now_ms; feed(0,frame+split,n-split);
        TEST_CHECK(reloads==before+1 && RefereeGetLastRxMs()==first);
    }
    n=referee_frame(frame,ID_game_result,1,5);
    const uint32_t starts[]={0u,UINT32_MAX-25u};
    for (unsigned i=0;i<2;++i) {
        restart(0); now_ms=starts[i]; feed(0,frame,5); first=now_ms;
        now_ms+=50u; before=reloads; feed(0,frame+5,n-5);
        TEST_CHECK(reloads==before+1 && RefereeGetLastRxMs()==first);
        now_ms=first+300u; TEST_CHECK(RefereeIsOnline());
        now_ms=first+301u; TEST_CHECK(!RefereeIsOnline());
        restart(0); now_ms=starts[i]; feed(0,frame,5); now_ms+=51u;
        before=reloads; feed(0,frame+5,n-5); TEST_CHECK(reloads==before);
    }
    restart(0); now_ms=1000; feed(0,frame,5);
    HAL_UART_ErrorCallback(&endpoints[0].handle); USARTProcessRx();
    before=reloads; feed(0,frame+5,n-5); TEST_CHECK(reloads==before);
    feed(0,frame,n); TEST_CHECK(reloads==before+1);
    restart(0); feed(0,frame,5); restart(0);
    before=reloads; feed(0,frame+5,n-5); TEST_CHECK(reloads==before);
    // 满队列丢新块会消耗连续性序号，下一轮尾段不能拼到旧头上。
    restart(0); const uint8_t noise=0;
    enqueue(0,&noise,1); enqueue(0,&noise,1); enqueue(0,frame,5); enqueue(0,&noise,1);
    before=reloads; USARTProcessRx(); feed(0,frame+5,n-5); TEST_CHECK(reloads==before);
    feed(0,frame,n); TEST_CHECK(reloads==before+1);
}
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
    daemon_configs[1].callback(daemon_configs[1].owner_id);
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
    referee_info_t *info=RefereeInit(&endpoints[0].handle);
    TEST_CHECK(info && VisionInit(&endpoints[1].handle));
    referee_cases(info); vision_cases();
    return 0;
}
