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
static uint32_t advance_on_vision_send; // 模拟命令计算后、发布前被任务抢占。
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)n; if(h==&huart5)dma=p; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t t)
{ (void)h;(void)p;(void)n;(void)t;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *p,uint16_t n) { return HAL_UART_Transmit(h,p,n,0); }
Vision_Recv_s *VisionInit(PlatformUartPort_t h) { (void)h;return &vision; }
void VisionSend(void) { now += advance_on_vision_send; advance_on_vision_send=0u; }
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
    if(argc>1 && !strcmp(argv[1],"pc-mouse-publish-expiry")) {
        pc_input(1u,0u,0u); pc_input(2u,0u,0u); pc_input(3u,0u,66u); // 缓存一份真实鼠标事件。
        const uint32_t origin=now;
        now=origin+70u; pc_input(3u,0u,66u); // 无线帧继续到达，重复 PC 心跳不产生新位移。
        now=origin+140u; pc_input(4u,0u,0u); // 新心跳保持键鼠在线，但不能更新旧事件时间。
        now=origin+150u; Gimbal_Upload_Data_s pose={0}; MessagePublishNow(feedback,&pose);
        advance_on_vision_send=1u; RobotCMDTask(); // 消费时恰好有效，发布前过期。
        Gimbal_Ctrl_Cmd_s dropped={0};
        CHECK(MessageReadSnapshot(gimbal,&dropped,sizeof(dropped),now,NULL)==MESSAGE_ERR_EMPTY);
        cycle(0); CHECK(observed_gimbal.gimbal_mode==GIMBAL_GYRO_MODE && observed_gimbal.yaw==0.0f); // 新周期从实际姿态接续，不复用过期位移。
        return 0;
    }
    if(argc>1 && !strcmp(argv[1],"pc-mouse-boundary")) {
        pc_input(1u,0u,0u); pc_input(2u,0u,0u); pc_input(3u,0u,66u);
        RemoteControlSnapshot_s earlier={0}; RemoteControlGetSnapshot(&earlier); // 消费者先取得第三份样本。
        pc_input(4u,0u,66u); // 随后到达的第四份不能混入已有快照。
        RemoteMouseBatch_t batch={0};
        CHECK(RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_OK && batch.x==66 && batch.count==1u);
        CHECK(RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_EMPTY);
        RemoteControlGetSnapshot(&earlier);
        CHECK(RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_OK && batch.x==66 && batch.count==1u);
        pc_input(5u,0u,66u); RemoteControlGetSnapshot(&earlier);
        CHECK(RemoteControlConsumeMouse(0u,&batch)==REMOTE_MOUSE_EMPTY); // 无有效快照时不能抢先消费。
        now+=70u; pc_input(5u,0u,66u); // 重复心跳只保持无线在线。
        now+=70u; pc_input(6u,0u,0u); // 保持键鼠在线，不改变旧位移时间。
        now+=11u; pc_input(7u,0u,0u); RemoteControlGetSnapshot(&earlier);
        CHECK(RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_EMPTY && batch.count==0u); // 超过 150ms 的旧事件被丢弃。
        pc_input(8u,0u,66u); // 缓存一份尚未消费的鼠标位移。
        const uint8_t centered[18]={0,4,32,0,1,8,0,0,0,0,8,0,0,0,0,0,0,4};
        memcpy(dma,centered,18); dma[5]|=(3u<<4)|(2u<<6); // 同一 PC 心跳下切换到遥控档。
        HAL_UARTEx_RxEventCallback(&huart5,18u); USARTProcessRx(); // 无线变化仍须清除旧位移。
        RemoteControlGetSnapshot(&earlier);
        CHECK(earlier.pc_valid && RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_EMPTY);
        pc_input(9u,0u,66u); now+=101u; pc_input(10u,0u,0u); // 即使离线事件尚未派发，接收间隔也已超无线期限。
        RemoteControlGetSnapshot(&earlier); CHECK(!earlier.pc_valid && EstopIsLatched()); // 调试失联锁存急停，重新连接仍需两帧松开。
        pc_input(11u,0u,0u); RemoteControlGetSnapshot(&earlier); CHECK(earlier.pc_valid);
        CHECK(RemoteControlConsumeMouse(earlier.pc_sample_sequence,&batch)==REMOTE_MOUSE_EMPTY); // 失联前的位移不能借新心跳恢复。
        RemoteControlDiagnostics_s before_gap={0}, after_gap={0};
        CHECK(RemoteControlGetDiagnostics(&before_gap));
        --now; pc_input(12u,0u,0u); ++now; // 人为注入来源时间回退的旧无线帧。
        CHECK(RemoteControlGetDiagnostics(&after_gap) && after_gap.rejected_fields==before_gap.rejected_fields+1u &&
              after_gap.last_valid_tick==before_gap.last_valid_tick); // 旧帧不能覆盖新接收时间。
        return 0;
    }
    if(argc>1 && !strcmp(argv[1],"pc-mouse-burst")) {
        pc_input(1u,0u,0u); pc_input(2u,0u,0u); // 两帧松开后才授权键鼠。
        pc_input(3u,1u,66u); pc_input(4u,1u,66u); // 两份新位移先于一次命令周期到达。
        cycle(0); CHECK(observed_gimbal.yaw==2.0f && observed_chassis.vx==300.0f);
        cycle(0); CHECK(observed_gimbal.yaw==2.0f); // 已消费的两份位移不能重复。
        pc_input(5u,1u,(uint16_t)-66); cycle(0); CHECK(observed_gimbal.yaw==1.0f);
        for (uint16_t heartbeat=6u; heartbeat<=13u; ++heartbeat) pc_input(heartbeat,1u,66u); // 容量内八份位移都应交付。
        cycle(0); CHECK(observed_gimbal.yaw==9.0f);
        for (uint16_t heartbeat=14u; heartbeat<=22u; ++heartbeat) pc_input(heartbeat,1u,66u); // 第九份位移触发有界队列溢出。
        cycle(0); CHECK(observed_gimbal.gimbal_mode==GIMBAL_ZERO_FORCE && observed_chassis.vx==300.0f);
        RemoteControlDiagnostics_s diagnostics;
        CHECK(RemoteControlGetDiagnostics(&diagnostics) && diagnostics.mouse_overflows==1u);
        pc_input(23u,1u,66u); cycle(0); // 溢出被消费后，从当前姿态接续新位移。
        CHECK(observed_gimbal.gimbal_mode==GIMBAL_GYRO_MODE && observed_gimbal.yaw==1.0f);
        return 0;
    }
    if(argc>1 && !strcmp(argv[1],"pc-vision")) {
        pc_input(1,0,0); pc_input(2,0,0); pc_input(3,1|(1<<Key_E)|(1<<Key_F),66); cycle(0);
        const uint32_t last=now;
        now=last+75; pc_input(3,1,66); // 重复 PC 心跳但维持无线在线。
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
        CHECK(observed_gimbal.yaw==1.0f); // 真实接收链中的鼠标位移只执行一次。
        cycle(0); CHECK(observed_gimbal.yaw==1.0f); // 没有新键鼠样本时不重复累加。
        const uint32_t last=now;
        now=last+75; pc_input(3,1|(1<<Key_E)|(1<<Key_F),66); // 无线有新帧，PC 心跳仍保持不变。
        now=last+150; pc_input(3,1|(1<<Key_E)|(1<<Key_F),66); cycle(0);
        CHECK(observed_chassis.vx==300);
        CHECK(observed_gimbal.yaw==1.0f); // 无线有新帧但 PC 心跳重复时也不能重放位移。
        MessageFreshness_t source_time={0}; Chassis_Ctrl_Cmd_s stored={0};
        CHECK(MessageReadSnapshot(chassis, &stored, sizeof(stored), now, &source_time)==MESSAGE_OK);
        CHECK(source_time.source_time_ms==last && source_time.published_time_ms==now && source_time.source_max_age_ms==REMOTE_PC_LEASE_MS); // 键鼠时间不能被新的无线帧覆盖。
        CHECK(MessageReadSnapshot(chassis, &stored, sizeof(stored), now+1u, NULL)==MESSAGE_ERR_EXPIRED); // 来源过期时，即使命令刚发布也不能继续执行。
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
