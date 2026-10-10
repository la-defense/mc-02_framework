#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "remote_control.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "message_center.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
UART_HandleTypeDef huart5,huart9;
static uint32_t now;
static uint8_t *dma;
static Vision_Recv_s vision;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)n; if(h==&huart5)dma=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;(void)t;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
Vision_Recv_s *VisionInit(UART_HandleTypeDef *h) { (void)h;return &vision; }
void VisionSend(void) {}
void VisionGetSnapshot(Vision_Recv_s *frame,Vision_Status_t *status)
{ *frame=vision;memset(status,0,sizeof(*status)); }
static void frame(uint8_t left,uint8_t right,uint16_t length,uint8_t mouse)
{
    const uint8_t centered[18]={0,4,32,0,1,8,0,0,0,0,0,0,0,0,0,0,0,4};
    memcpy(dma,centered,18); dma[5]|=(right<<4)|(left<<6); dma[12]=mouse;
    HAL_UARTEx_RxEventCallback(&huart5,length); USARTProcessRx();
}
static Subscriber_t *gimbal,*shoot,*chassis;
static Publisher_t *feedback;
static void cycle(uint8_t neutral)
{
    Gimbal_Upload_Data_s pose={0}; PubPushMessage(feedback,&pose); RobotCMDTask();
    Gimbal_Ctrl_Cmd_s g; Shoot_Ctrl_Cmd_s s; Chassis_Ctrl_Cmd_s c;
    CHECK(SubGetMessage(gimbal,&g) && SubGetMessage(shoot,&s) && SubGetMessage(chassis,&c));
    if(neutral) {
        CHECK(g.gimbal_mode==GIMBAL_ZERO_FORCE && g.yaw==0 && g.pitch==0 && g.yaw_vel==0 && g.pitch_acc==0);
        CHECK(s.shoot_mode==SHOOT_OFF && s.friction_mode==FRICTION_OFF && s.load_mode==LOAD_STOP);
        CHECK(c.chassis_mode==CHASSIS_ZERO_FORCE && c.vx==0 && c.vy==0);
    }
}
int main(void)
{
    RobotSafetyInit(); RobotCMDInit();
    feedback=PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));
    gimbal=SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrl_Cmd_s));
    shoot=SubRegister("shoot_cmd",sizeof(Shoot_Ctrl_Cmd_s));
    chassis=SubRegister("chassis_cmd",sizeof(Chassis_Ctrl_Cmd_s));
    cycle(1); CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    frame(3,3,17,0); cycle(1); CHECK(!RemoteControlIsOnline());
    RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    for(now=0;now<=1000;now+=10) { frame(3,3,18,0); cycle(1); DaemonTask(); }
    CHECK(RemoteControlIsOnline() && !EstopIsLatched() && RobotSafetyGetState()==ROBOT_SAFE);
    for(;now<=2020;now+=10) { frame(2,2,18,0); cycle(0); DaemonTask(); }
    CHECK(RobotSafetyIsReady());
    const uint32_t accepted=remote_control_diag.accepted_frames;
    for(unsigned i=0;i<11;++i) { now+=10;frame(2,2,18,2);DaemonTask();cycle(i>=9); }
    CHECK(remote_control_diag.accepted_frames==accepted && !RemoteControlIsOnline() && EstopIsLatched());
    frame(2,2,18,0);cycle(1);CHECK(RemoteControlIsOnline() && EstopIsLatched());
    for(unsigned state=0;state<3;++state) {
        RobotSafetyInit(); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
        if(state==1)RobotSafetySetTaskHealthy(0);
        if(state==2)EstopRequest(ESTOP_REASON_MANUAL);
        now+=10;frame(2,3,18,0);cycle(1);CHECK(!RobotSafetyIsReady());
    }
    return 0;
}
