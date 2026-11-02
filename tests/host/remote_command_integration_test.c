#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "remote_control.h"
#include "bsp_usart.h"
#include "robot_def.h"
#include "daemon.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); exit(1); } } while(0)
UART_HandleTypeDef huart5,huart9;
static DMA_HandleTypeDef huart5_dma, huart9_dma;
static uint32_t now;
static uint8_t *dma;
static Vision_Recv_s vision;
static Vision_Status_t vision_status;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)n; if(h==&huart5)dma=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;(void)t;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
Vision_Recv_s *VisionInit(PlatformUartPort_t h) { (void)h;return &vision; }
void VisionSend(void) {}
void VisionGetSnapshot(Vision_Recv_s *frame,Vision_Status_t *status)
{ *frame=vision;*status=vision_status; }
static void frame(uint8_t left,uint8_t right,uint16_t length,uint8_t mouse)
{
    const uint8_t centered[18]={0,4,32,0,1,8,0,0,0,0,0,0,0,0,0,0,0,4};
    memcpy(dma,centered,18); dma[5]|=(right<<4)|(left<<6); dma[12]=mouse;
    HAL_UARTEx_RxEventCallback(&huart5,length); USARTProcessRx();
}
static void pc_input(uint16_t heartbeat,uint16_t keys,uint16_t x)
{
    const uint8_t centered[18]={0,4,32,0,1,8,0,0,0,0,0,0,0,0,0,0,0,4};
    memcpy(dma,centered,18); dma[5]|=(3<<4)|(1<<6);
    dma[6]=(uint8_t)x; dma[7]=(uint8_t)(x>>8);
    dma[10]=(uint8_t)heartbeat; dma[11]=(uint8_t)(heartbeat>>8);
    dma[14]=(uint8_t)keys; dma[15]=(uint8_t)(keys>>8);
    HAL_UARTEx_RxEventCallback(&huart5,18); USARTProcessRx();
}
static MessageSubscriber_t *gimbal,*shoot,*chassis;
static MessagePublisher_t *feedback;
static Gimbal_Ctrl_Cmd_s observed_gimbal;
static Shoot_Ctrl_Cmd_s observed_shoot;
static Chassis_Ctrl_Cmd_s observed_chassis;
static void cycle(uint8_t neutral)
{
    Gimbal_Upload_Data_s pose={0}; MessagePublishNow(feedback, &pose); RobotCMDTask();
    Gimbal_Ctrl_Cmd_s g; Shoot_Ctrl_Cmd_s s; Chassis_Ctrl_Cmd_s c;
    CHECK(MessageReadSnapshot(gimbal, &g, sizeof(g), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &s, sizeof(s), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(chassis, &c, sizeof(c), HAL_GetTick(), NULL) == MESSAGE_OK);
    observed_gimbal=g; observed_shoot=s; observed_chassis=c;
    if(neutral) {
        CHECK(g.gimbal_mode==GIMBAL_ZERO_FORCE && g.yaw==0 && g.pitch==0 && g.yaw_vel==0 && g.pitch_acc==0);
        CHECK(s.shoot_mode==SHOOT_OFF && s.friction_mode==FRICTION_OFF && s.load_mode==LOAD_STOP);
        CHECK(c.chassis_mode==CHASSIS_ZERO_FORCE && c.vx==0 && c.vy==0);
    }
}
int main(int argc,char **argv)
{
    huart5.hdmarx=&huart5_dma; huart9.hdmarx=&huart9_dma;
    RobotSafetyInit(); RobotCMDInit();
    feedback=MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    gimbal=MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_COMMAND, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    shoot=MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_SHOOT);
    chassis=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_CHASSIS);
    cycle(1); CHECK(!RemoteControlIsOnline() && EstopIsLatched());
    frame(3,3,17,0); cycle(1); CHECK(!RemoteControlIsOnline());
    RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    for(now=0;now<=1000;now+=10) { frame(3,3,18,0); cycle(1); DaemonTask(); }
    CHECK(RemoteControlIsOnline() && !EstopIsLatched() && RobotSafetyGetState()==ROBOT_SAFE);
    for(;now<=2020;now+=10) { frame(2,2,18,0); cycle(0); DaemonTask(); }
    CHECK(RobotSafetyIsReady());
    if(argc>1 && !strcmp(argv[1],"pc-vision")) {
        pc_input(1,0,0); pc_input(2,0,0); pc_input(3,1|(1<<Key_E)|(1<<Key_F),66); cycle(0);
        const uint32_t last=now;
        now=last+150; pc_input(3,1,66);
        vision=(Vision_Recv_s){.mode=2,.yaw=.1f,.pitch=.1f,.yaw_vel=.2f,.pitch_acc=-.3f};
        vision_status.online=1; vision_status.last_rx_ms=now;
        cycle(0); CHECK(observed_shoot.load_mode==LOAD_BURSTFIRE && observed_gimbal.yaw_vel>0);
        now=last+151; cycle(0);
        CHECK(observed_chassis.chassis_mode==CHASSIS_ZERO_FORCE && observed_chassis.vx==0);
        CHECK(observed_shoot.friction_mode==FRICTION_ON && observed_shoot.load_mode==LOAD_BURSTFIRE);
        CHECK(observed_gimbal.yaw_vel>0 && observed_gimbal.pitch_acc<0);
        CHECK(RobotSafetyIsReady() && !EstopIsLatched());
        vision.mode=1; cycle(0); CHECK(observed_shoot.load_mode==LOAD_STOP && observed_shoot.friction_mode==FRICTION_ON);
        vision.mode=0; cycle(1);
        pc_input(4,1,0); cycle(1);
        pc_input(5,0,0); pc_input(6,0,0); pc_input(7,1,0); cycle(0);
        CHECK(observed_chassis.vx==300 && observed_shoot.load_mode==LOAD_STOP);
        CHECK(observed_gimbal.yaw==0 && observed_gimbal.yaw_vel==0); /* New pose, no old mouse. */
        vision.mode=2; vision_status.last_rx_ms=now; RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); cycle(1);
        RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF); RobotSafetyRequestEnable(); EstopRequest(ESTOP_REASON_MANUAL); cycle(1);
        return 0;
    }
    if(argc>1 && !strcmp(argv[1],"pc-command")) {
        pc_input(1,0,0); pc_input(2,0,0); pc_input(3,1|(1<<Key_E)|(1<<Key_F),66);
        cycle(0); CHECK(RobotSafetyIsReady());
        CHECK(observed_chassis.vx==300);
        CHECK(observed_shoot.friction_mode==FRICTION_ON);
        const uint32_t last=now;
        now=last+150; pc_input(3,1|(1<<Key_E)|(1<<Key_F),66); cycle(0);
        CHECK(observed_chassis.vx==300);
        now=last+151; cycle(1); /* No new frame: command task must revoke. */
        CHECK(RemoteControlIsOnline() && RobotSafetyIsReady() && !EstopIsLatched());
        frame(2,3,18,0); cycle(0); /* RC source remains usable. */
        CHECK(observed_gimbal.gimbal_mode==GIMBAL_GYRO_MODE);
        pc_input(4,1,0); cycle(1); /* Held input cannot re-arm. */
        pc_input(5,0,0); pc_input(6,0,0); pc_input(7,1,0); cycle(0);
        CHECK(observed_chassis.vx==300);
        CHECK(observed_shoot.friction_mode==FRICTION_OFF && observed_shoot.load_mode==LOAD_STOP);
        return 0;
    }
    RemoteControlDiagnostics_s remote_diagnostics;
    CHECK(RemoteControlGetDiagnostics(&remote_diagnostics));
    const uint32_t accepted=remote_diagnostics.accepted_frames;
    for(unsigned i=0;i<11;++i) { now+=10;frame(2,2,18,2);DaemonTask();cycle(i>=9); }
    CHECK(RemoteControlGetDiagnostics(&remote_diagnostics));
    CHECK(remote_diagnostics.accepted_frames==accepted && !RemoteControlIsOnline() && EstopIsLatched());
    frame(2,2,18,0);cycle(1);CHECK(RemoteControlIsOnline() && EstopIsLatched());
    for(unsigned state=0;state<3;++state) {
        RobotSafetyInit(); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
        if(state==1)RobotSafetySetTaskHealthy(0);
        if(state==2)EstopRequest(ESTOP_REASON_MANUAL);
        now+=10;frame(2,3,18,0);cycle(1);CHECK(!RobotSafetyIsReady());
    }
    return 0;
}
