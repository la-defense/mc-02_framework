#include <stdio.h>
#include <string.h>
#include <math.h>
#include "robot_def.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "remote_control.h"
#include "message_center.h"
#include "vision_control.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL:%d %s\n",__LINE__,#c); return 1; } } while(0)
UART_HandleTypeDef huart5, huart9;
static RC_ctrl_t remote[2];
static uint8_t remote_online=1;
static uint32_t tick=100;
static Vision_Recv_s wire;
static Vision_Status_t status;
static unsigned snapshots;
uint32_t HAL_GetTick(void) { return tick; }
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *h) { (void)h; return remote; }
uint8_t RemoteControlIsOnline(void) { return remote_online; }
Vision_Recv_s *VisionInit(UART_HandleTypeDef *h) { (void)h; return &wire; }
void VisionSend(void) {}
void VisionGetSnapshot(Vision_Recv_s *frame,Vision_Status_t *link) { *frame=wire; *link=status; ++snapshots; }
int main(void)
{
    RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1);
    RobotSafetySetCalibValid(1); RobotSafetyRequestEnable(); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    RobotCMDInit();
    Publisher_t *feedback=PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));
    Subscriber_t *gimbal=SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrl_Cmd_s));
    Subscriber_t *shoot=SubRegister("shoot_cmd",sizeof(Shoot_Ctrl_Cmd_s));
    Gimbal_Upload_Data_s pose={0}; pose.gimbal_imu_data.YawTotalAngle=12; pose.gimbal_imu_data.Pitch=3;
    Gimbal_Ctrl_Cmd_s command; Shoot_Ctrl_Cmd_s shooting;
    remote[0].rc.switch_left=RC_SW_DOWN; remote[0].rc.switch_right=RC_SW_MID;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command)); CHECK(command.gimbal_mode==GIMBAL_GYRO_MODE);
    remote[0].rc.switch_left=RC_SW_MID; tick+=5;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command)); CHECK(SubGetMessage(shoot,&shooting));
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE);
    CHECK(command.yaw==0 && command.pitch==0 && command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
    CHECK(snapshots==2);
#if VISION_LIMITS_CONFIGURED
    /* Whole production task + production safety + production message queues. */
    remote[0].rc.switch_left=RC_SW_DOWN;
    remote[0].rc.dial=-600;
    wire.mode=2; wire.yaw=0.25f; wire.pitch=0.1f;
    wire.yaw_vel=0.2f; wire.yaw_acc=0.3f; wire.pitch_vel=-0.2f; wire.pitch_acc=-0.3f;
    status.online=1;
    unsigned before=snapshots;
    tick+=5; status.last_rx_ms=tick;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(snapshots==before+1);
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.gimbal_mode==GIMBAL_GYRO_MODE && fabsf(command.yaw-14.323945f)<0.0001f);
    CHECK(fabsf(command.pitch-10.729578f)<0.0001f && command.yaw_vel>0 && command.pitch_acc<0);
    CHECK(shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==10);
    wire.mode=1; tick+=5; status.last_rx_ms=tick;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.yaw_vel>0 && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
    CHECK(shooting.friction_mode==FRICTION_ON);
    wire.mode=0; tick+=5;
    remote[0].rc.rocker_l_=100; remote[0].rc.rocker_l1=-100;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(fabsf(command.yaw-12.2f)<0.0001f && fabsf(command.pitch-2.9f)<0.0001f);
    CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    CHECK(shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
    remote[0].rc.rocker_l_=remote[0].rc.rocker_l1=0;
    /* Each invalidation follows a real active vision cycle. */
    for (unsigned reason=0;reason<6;reason++) {
        status.online=1; wire.mode=2; wire.pitch=0.1f; tick+=5; status.last_rx_ms=tick;
        PubPushMessage(feedback,&pose); RobotCMDTask();
        CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
        CHECK(command.yaw_vel>0 && shooting.shoot_rate==10);
        tick+=5;
        if (reason==0) status.online=0;
        if (reason==1) tick=status.last_rx_ms+51;
        if (reason==2) wire.pitch=1.0f;
        if (reason==3) wire.pitch=NAN;
        if (reason==4) wire.mode=0;
        if (reason==5) wire.mode=3;
        PubPushMessage(feedback,&pose); RobotCMDTask();
        CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
        CHECK(command.yaw==12 && command.pitch==3 && command.gimbal_mode==GIMBAL_GYRO_MODE);
        CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
        CHECK(shooting.shoot_rate==8 && shooting.load_mode==LOAD_BURSTFIRE);
    }
    /* Missing/non-finite handoff feedback must not leave the old target active. */
    for (unsigned reason=0;reason<3;reason++) {
        wire.mode=2; wire.pitch=0.1f; status.online=1; tick+=5; status.last_rx_ms=tick;
        PubPushMessage(feedback,&pose); RobotCMDTask();
        CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
        wire.mode=0; tick+=5;
        Gimbal_Upload_Data_s invalid=pose;
        if (reason==1) invalid.gimbal_imu_data.Pitch=NAN;
        if (reason==2) invalid.gimbal_imu_data.YawTotalAngle=INFINITY;
        if (reason) PubPushMessage(feedback,&invalid);
        RobotCMDTask();
        CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
        CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0);
        CHECK(command.yaw_vel==0 && command.pitch_acc==0);
        CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP);
    }
    /* Keyboard mappings remain current after vision is revoked. */
    remote[0].rc.switch_left=RC_SW_UP;
    remote[0].key_count[KEY_PRESS][Key_E]=3; remote[0].key_count[KEY_PRESS][Key_F]=1;
    wire.mode=2; status.online=1; tick+=5; status.last_rx_ms=tick;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    wire.mode=0; remote[0].mouse.x=66; remote[0].mouse.y=-66; tick+=5;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.yaw==13 && command.pitch==2 && command.yaw_vel==0 && command.pitch_acc==0);
    CHECK(shooting.shoot_rate==8 && shooting.load_mode==LOAD_BURSTFIRE);
    /* No selected manual source after mode 2: complete output revocation. */
    wire.mode=2; tick+=5; status.last_rx_ms=tick;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    wire.mode=0; remote[0].rc.switch_left=RC_SW_MID; tick+=5;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.yaw_vel==0);
    CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
#else
    /* Actual default config refuses otherwise valid mode 2. */
    wire.mode=2; wire.yaw=0.25f; wire.pitch=0.1f; status.online=1; status.last_rx_ms=tick;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && shooting.shoot_mode==SHOOT_OFF);
#endif
    /* Safety overrides valid sources in every non-READY state. */
    for (unsigned state=0;state<4;state++) {
        RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
        if (state==1) RobotSafetyRequestCalib(1);
        if (state==2) RobotSafetySetTaskHealthy(0);
        if (state==3) EstopRequest(ESTOP_REASON_MANUAL);
        remote[0].rc.switch_left=RC_SW_DOWN; remote[0].rc.switch_right=RC_SW_MID; remote[0].rc.dial=-600;
        status.online=1; wire.mode=2; wire.pitch=0.1f; tick+=5; status.last_rx_ms=tick;
        PubPushMessage(feedback,&pose); RobotCMDTask();
        CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
        CHECK(!RobotSafetyIsReady());
        CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0 && command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
        CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
    }
    RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    remote_online=0; tick+=5;
    PubPushMessage(feedback,&pose); RobotCMDTask();
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && shooting.shoot_mode==SHOOT_OFF);
    puts("PASS: production command task revokes outputs without a source");
    return 0;
}
