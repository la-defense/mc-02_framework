#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "remote_control.h"
#include "daemon.h"
#include "robot_safety.h"
#include "bsp_usart.h"
#include "remote_safety.h"
#if MC02_BENCH_RC_DIAGNOSTICS
#include "remote_bench.h"
#endif
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
UART_HandleTypeDef huart5, huart9;
static uint32_t now;
static uint8_t *dma_buffer;
extern volatile uint32_t usart_rx_oversize_total;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)n; if (h == &huart5) dma_buffer = p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ return HAL_UART_Transmit(h,p,n,0); }
static void encode(uint8_t b[18], const uint16_t ch[5], uint8_t left, uint8_t right)
{
    memset(b, 0, 18);
    b[0] = ch[0]; b[1] = (ch[0] >> 8) | (ch[1] << 3);
    b[2] = (ch[1] >> 5) | (ch[2] << 6); b[3] = ch[2] >> 2;
    b[4] = (ch[2] >> 10) | (ch[3] << 1);
    b[5] = (ch[3] >> 7) | (right << 4) | (left << 6);
    b[16] = ch[4]; b[17] = ch[4] >> 8;
}
static void receive(const uint8_t b[18], uint16_t size)
{
    memcpy(dma_buffer, b, 18);
    HAL_UARTEx_RxEventCallback(&huart5, size);
    USARTProcessRx();
}
int main(int argc, char **argv)
{
    RobotSafetyInit();
#if MC02_BENCH_RC_DIAGNOSTICS
    RemoteBenchInit(&huart5);
    RemoteBenchTask(); CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    uint8_t frame[18]; const uint16_t center[5]={1024,1024,1024,1024,1024};
    encode(frame,center,3,3);
    for(now=0;now<=1000;now+=10) { receive(frame,18); RemoteBenchTask(); DaemonTask(); }
    CHECK(RemoteControlIsOnline() && !EstopIsLatched() && RobotSafetyGetState()==ROBOT_SAFE);
    encode(frame,center,2,2);
    for(now=1010;now<=2020;now+=10) {
        RobotSafetyRequestEnable(); receive(frame,18); RemoteBenchTask(); DaemonTask();
        CHECK(!RobotSafetyIsReady());
    }
    const Robot_Fault_e faults[]={ROBOT_FAULT_MOTOR_OFFLINE,ROBOT_FAULT_CAN_BUSOFF,ROBOT_FAULT_TILT};
    for(unsigned i=0;i<3;++i) {
        RobotSafetyRequestEnable(); RobotFaultReport(faults[i]);
        now+=10; receive(frame,18); RemoteBenchTask(); DaemonTask();
        CHECK(RobotSafetyGetState()==ROBOT_FAULT && !RobotSafetyIsReady());
        RobotFaultClear(faults[i]); now+=10; receive(frame,18); RemoteBenchTask(); DaemonTask();
        CHECK(RobotSafetyGetState()==ROBOT_SAFE && !RobotSafetyIsReady());
    }
    for(unsigned i=0;i<11;++i) { now+=10; DaemonTask(); RemoteBenchTask(); }
    CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    receive(frame,18); RemoteBenchTask(); CHECK(RemoteControlIsOnline() && EstopIsLatched());
    CHECK(!RobotSafetyIsReady());
    return 0;
#endif
    RC_ctrl_t *rc = RemoteControlInit(&huart5);
    CHECK(rc != NULL && dma_buffer != NULL);
    CHECK(!RemoteControlIsOnline());
    RobotSafetyUpdate();
    CHECK(EstopIsLatched());
    CHECK(RobotSafetyGetState() == ROBOT_ESTOP);
    if (argc > 1 && !strcmp(argv[1], "length"))
    {
        uint8_t b[18]; const uint16_t center[5] = {1024,1024,1024,1024,1024};
        encode(b, center, 3, 3);
        RC_ctrl_t before[2]; memcpy(before, rc, sizeof(before));
        for (unsigned n=0; n<18; ++n) {
            receive(b, n);
            CHECK(!RemoteControlIsOnline());
            CHECK(!memcmp(before, rc, sizeof(before)));
        }
        receive(b, 19);
        receive(b, 65535);
        CHECK(!RemoteControlIsOnline());
        CHECK(usart_rx_oversize_total == 2);
        CHECK(remote_control_diag.rejected_length == 17);
        CHECK(!memcmp(before, rc, sizeof(before)));
        receive(b, 18);
        CHECK(RemoteControlIsOnline());
        CHECK(EstopIsLatched());
    }
    if (argc > 1 && !strcmp(argv[1], "fields"))
    {
        uint8_t b[18]; uint16_t ch[5] = {1024,1024,1024,1024,1024};
        encode(b, ch, 3, 3); receive(b, 18);
        RC_ctrl_t before[2]; memcpy(before, rc, sizeof(before));
        for (unsigned i=0; i<5; ++i) {
            ch[i]=363; encode(b,ch,3,3); b[14]=1; receive(b,18);
            CHECK(!memcmp(before,rc,sizeof(before)));
            ch[i]=1685; encode(b,ch,3,3); receive(b,18);
            CHECK(!memcmp(before,rc,sizeof(before)));
            ch[i]=1024;
        }
        encode(b,ch,0,3); receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        encode(b,ch,3,0); receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        for (unsigned i=12; i<=13; ++i) {
            encode(b,ch,3,3); b[i]=2; receive(b,18); CHECK(!memcmp(before,rc,sizeof(before)));
        }
        for (unsigned i=0; i<11; ++i) { now += 10; receive(b,18); DaemonTask(); }
        CHECK(!RemoteControlIsOnline()); CHECK(EstopIsLatched());
        RC_ctrl_t empty[2] = {0}; CHECK(!memcmp(empty,rc,sizeof(empty)));
        receive(b,18); CHECK(!RemoteControlIsOnline());
        CHECK(remote_control_diag.offline_events == 1);
        CHECK(remote_control_diag.offline_tick - remote_control_diag.offline_last_valid_tick == 110);
        for (unsigned edge=0; edge<2; ++edge) {
            for(unsigned i=0; i<5; ++i) ch[i]=edge ? 1684 : 364;
            encode(b,ch,1,2); receive(b,18);
            CHECK(RemoteControlIsOnline()); CHECK(rc[TEMP].rc.dial == (edge ? 660 : -660));
            CHECK(rc[TEMP].rc.rocker_l_ == (edge ? 660 : -660));
        }
    }
    if (argc > 1 && !strcmp(argv[1], "queue-keys")) {
        uint8_t b[18]; uint16_t ch[5]={1100,900,1200,800,1024};
        encode(b,ch,1,2); b[6]=0xfe; b[7]=0xff; b[8]=0x23; b[9]=1; b[12]=b[13]=1;
        b[14]=1; memcpy(dma_buffer,b,18); HAL_UARTEx_RxEventCallback(&huart5,18);
        memset(dma_buffer,0xff,18); USARTProcessRx();
        CHECK(RemoteControlIsOnline()); CHECK(rc[0].rc.rocker_r_==76);
        CHECK(rc[0].rc.rocker_r1==-124 && rc[0].rc.rocker_l_==176 && rc[0].rc.rocker_l1==-224);
        CHECK(rc[0].mouse.x==-2 && rc[0].mouse.y==291 && rc[0].mouse.press_r==1);
        CHECK(rc[0].key_count[KEY_PRESS][Key_W]==1);
        receive(b,18); CHECK(rc[0].key_count[KEY_PRESS][Key_W]==1);
        b[14]=0; b[12]=2; receive(b,18); /* Invalid release doesn't alter edges. */
        b[14]=1; b[12]=1; receive(b,18); CHECK(rc[0].key_count[KEY_PRESS][Key_W]==1);
        b[14]=0; receive(b,18); b[14]=1|(1<<Key_Ctrl); receive(b,18);
        CHECK(rc[0].key_count[KEY_PRESS_WITH_CTRL][Key_W]==1);
        b[14]=0; receive(b,18); b[14]=1|(1<<Key_Shift); receive(b,18);
        CHECK(rc[0].key_count[KEY_PRESS_WITH_SHIFT][Key_W]==1);
        CHECK(remote_control_diag.accepted_frames==7 && remote_control_diag.rejected_fields==1);
    }
    if (argc > 1 && !strcmp(argv[1], "gestures")) {
        RemoteSafetyGestures_s holds; RemoteSafetyGesturesInit(&holds);
        RC_ctrl_t input={0}; input.rc.switch_left=input.rc.switch_right=RC_SW_MID;
        RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
        const uint32_t starts[]={0,UINT32_MAX-500};
        for(unsigned i=0;i<2;++i) {
            EstopRequest(ESTOP_REASON_MANUAL); RemoteSafetyGesturesInit(&holds);
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i],1);
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+999u,1); CHECK(EstopIsLatched());
            RemoteSafetyGesturesUpdate(&holds,&input,1,starts[i]+1000u,1); CHECK(!EstopIsLatched());
            RobotSafetyUpdate(); CHECK(RobotSafetyGetState()==ROBOT_SAFE);
        }
        input.rc.switch_left=input.rc.switch_right=RC_SW_DOWN;
        RemoteSafetyGesturesInit(&holds);
        RobotSafetyRequestEnable(); /* Diagnostic must revoke even an earlier request. */
        RemoteSafetyGesturesUpdate(&holds,&input,1,0,0);
        RemoteSafetyGesturesUpdate(&holds,&input,1,1000,0);
        RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        RemoteSafetyGesturesUpdate(&holds,&input,1,1001,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,2001,1); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        input.rc.switch_left=input.rc.switch_right=RC_SW_UP;
        RemoteSafetyGesturesUpdate(&holds,&input,1,2100,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,3100,1); RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
        EstopRequest(ESTOP_REASON_MANUAL); input.rc.switch_left=input.rc.switch_right=RC_SW_MID;
        RemoteSafetyGesturesUpdate(&holds,&input,1,4000,1);
        RemoteSafetyGesturesUpdate(&holds,&input,0,4500,1);
        RemoteSafetyGesturesUpdate(&holds,&input,1,5000,1); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,5999,1); CHECK(EstopIsLatched());
        RemoteSafetyGesturesUpdate(&holds,&input,1,6000,1); CHECK(!EstopIsLatched());
        input.rc.dial=301; RemoteSafetyGesturesUpdate(&holds,&input,1,6100,1); CHECK(EstopIsLatched());
    }
    return 0;
}
