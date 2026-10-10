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
/* Subscribe to production messages rather than inspecting task-private state. */
static int shoot_cycle(Publisher_t *feedback, Subscriber_t *gimbal, Subscriber_t *shoot,
                       Gimbal_Upload_Data_s *pose, shoot_mode_e enabled,
                       friction_mode_e friction, loader_mode_e load, float rate, int vision)
{
    Gimbal_Ctrl_Cmd_s command; Shoot_Ctrl_Cmd_s shooting;
    unsigned before=snapshots;
    PubPushMessage(feedback,pose); RobotCMDTask();
    CHECK(snapshots==before+1);
    CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting));
    CHECK(shooting.shoot_mode==enabled && shooting.friction_mode==friction);
    CHECK(shooting.load_mode==load && shooting.shoot_rate==rate);
    if (vision) CHECK(command.yaw_vel>0 && command.pitch_acc<0);
    else CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    return 0;
}

static int shoot_switches(Publisher_t *feedback, Subscriber_t *gimbal, Subscriber_t *shoot,
                          Gimbal_Upload_Data_s *pose)
{
#define CYCLE(on, friction, load, rate, vision) CHECK(!shoot_cycle(feedback,gimbal,shoot,pose,on,friction,load,rate,vision))
    /* RC -> keyboard -> RC, including changes to the inactive source. */
    remote[0].rc.dial=-600;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,0);
    remote[0].rc.switch_left=RC_SW_UP;
    remote[0].key_count[KEY_PRESS][Key_E]=1;
    remote[0].key_count[KEY_PRESS][Key_F]=0;
    CYCLE(SHOOT_ON,FRICTION_OFF,LOAD_1_BULLET,8,0);
    remote[0].rc.dial=0;
    remote[0].rc.switch_left=RC_SW_DOWN;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,0);
#if VISION_LIMITS_CONFIGURED
    wire=(Vision_Recv_s){.mode=2,.yaw=.25f,.pitch=.1f,.yaw_vel=.2f,.pitch_acc=-.3f};
    status.online=1; status.last_rx_ms=tick;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
    /* Changing the dial while vision owns shooting must not leak into it. */
    remote[0].rc.dial=-600;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
    wire.mode=1;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_STOP,0,1);
    wire.mode=0;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,0);
    remote[0].rc.switch_left=RC_SW_UP;
    remote[0].key_count[KEY_PRESS][Key_E]=0;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,0);
    wire.mode=2;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
    /* E/F edges received during takeover select the current manual command. */
    remote[0].key_count[KEY_PRESS][Key_E]=2;
    remote[0].key_count[KEY_PRESS][Key_F]=1;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
    wire.mode=0;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
    /* Unsigned frame age: 50ms allowed, 51ms revoked, including wrap. */
    for (unsigned wrap=0;wrap<2;++wrap) {
        tick=wrap ? UINT32_MAX-20 : 200;
        wire.mode=2; status.last_rx_ms=tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
        tick+=50;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
        ++tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
    }
    /* Reject each non-finite field after a valid takeover, not just pitch. */
    for (unsigned field=0;field<6;++field) for (unsigned inf=0;inf<2;++inf) {
        wire=(Vision_Recv_s){.mode=2,.yaw=.25f,.pitch=.1f,.yaw_vel=.2f,.pitch_acc=-.3f};
        status.last_rx_ms=tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
        float bad=inf ? INFINITY : NAN;
        switch (field) {
        case 0: wire.yaw=bad; break; case 1: wire.pitch=bad; break;
        case 2: wire.yaw_vel=bad; break; case 3: wire.yaw_acc=bad; break;
        case 4: wire.pitch_vel=bad; break; default: wire.pitch_acc=bad; break;
        }
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
    }
    wire=(Vision_Recv_s){.mode=2,.yaw=.25f,.pitch=.1f,.yaw_vel=.2f,.pitch_acc=-.3f};
    status.last_rx_ms=tick;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,10,1);
    status.online=0;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
#else
    /* A real default (unconfigured) limit cannot grant a shooting takeover. */
    wire=(Vision_Recv_s){.mode=2,.yaw=.25f,.pitch=.1f};
    status.online=1; status.last_rx_ms=tick;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,0);
#endif
    wire.mode=0; remote[0].rc.switch_left=RC_SW_MID;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,0,0);
    puts("PASS: production shooting selects current source and revokes stale vision");
    return 0;
#undef CYCLE
}
int main(int argc, char **argv)
{
    RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1);
    RobotSafetySetCalibValid(1); RobotSafetyRequestEnable(); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    RobotCMDInit();
    if (argc > 1 && !strcmp(argv[1], "gesture-zero")) {
        EstopRequest(ESTOP_REASON_MANUAL);
        remote[0].rc.switch_left=remote[0].rc.switch_right=RC_SW_MID;
        tick=0; RobotCMDTask();
        tick=999; RobotCMDTask(); CHECK(EstopIsLatched());
        tick=1000; RobotCMDTask(); CHECK(!EstopIsLatched());
        CHECK(RobotSafetyGetState()==ROBOT_SAFE);
        return 0;
    }
    Publisher_t *feedback=PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));
    Subscriber_t *gimbal=SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrl_Cmd_s));
    Subscriber_t *shoot=SubRegister("shoot_cmd",sizeof(Shoot_Ctrl_Cmd_s));
    Gimbal_Upload_Data_s pose={0}; pose.gimbal_imu_data.YawTotalAngle=12; pose.gimbal_imu_data.Pitch=3;
    Gimbal_Ctrl_Cmd_s command; Shoot_Ctrl_Cmd_s shooting;
    remote[0].rc.switch_left=RC_SW_DOWN; remote[0].rc.switch_right=RC_SW_MID;
    if (argc > 1 && !strcmp(argv[1], "shoot-switches"))
        return shoot_switches(feedback,gimbal,shoot,&pose);
    if (argc > 1 && !strcmp(argv[1], "faults")) {
        Subscriber_t *chassis=SubRegister("chassis_cmd",sizeof(Chassis_Ctrl_Cmd_s));
        Chassis_Ctrl_Cmd_s driving;
        const Robot_Fault_e faults[]={ROBOT_FAULT_MOTOR_OFFLINE,ROBOT_FAULT_CAN_BUSOFF,ROBOT_FAULT_TILT};
        remote[0].rc.dial=-600;
        remote[0].rc.rocker_l_=100; remote[0].rc.rocker_l1=-100;
        remote[0].rc.rocker_r_=50; remote[0].rc.rocker_r1=-50;
        for (unsigned i=0;i<3;++i) {
            wire.mode=2; wire.yaw=.25f; wire.pitch=.1f;
            wire.yaw_vel=.2f; wire.yaw_acc=.3f; wire.pitch_vel=-.2f; wire.pitch_acc=-.3f;
            status.online=1; tick+=5; status.last_rx_ms=tick;
            PubPushMessage(feedback,&pose); RobotCMDTask();
            CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting) && SubGetMessage(chassis,&driving));
            CHECK(RobotSafetyIsReady() && command.gimbal_mode==GIMBAL_GYRO_MODE);
            CHECK(driving.vx!=0 && driving.vy!=0 && shooting.load_mode==LOAD_BURSTFIRE);
#if VISION_LIMITS_CONFIGURED
            CHECK(command.yaw_vel>0 && shooting.shoot_rate==10);
#else
            CHECK(command.yaw_vel==0 && shooting.shoot_rate==8);
#endif
            RobotFaultReport(faults[i]); tick+=5; status.last_rx_ms=tick;
            PubPushMessage(feedback,&pose); RobotCMDTask();
            CHECK(RobotSafetyGetState()==ROBOT_FAULT);
            CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting) && SubGetMessage(chassis,&driving));
            CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0);
            CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
            CHECK(driving.chassis_mode==CHASSIS_ZERO_FORCE && driving.vx==0 && driving.vy==0 && driving.wz==0 && driving.offset_angle==0);
            CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
            /* Recovery anchors to this cycle's pose and current manual inputs. */
            pose.gimbal_imu_data.YawTotalAngle=20+i; pose.gimbal_imu_data.Pitch=4;
            wire.mode=0; remote[0].rc.rocker_r_=60+i;
            RobotFaultClear(faults[i]); tick+=5;
            PubPushMessage(feedback,&pose); RobotCMDTask();
            CHECK(RobotSafetyIsReady());
            CHECK(SubGetMessage(gimbal,&command) && SubGetMessage(shoot,&shooting) && SubGetMessage(chassis,&driving));
            CHECK(fabsf(command.yaw-(20.2f+i))<.0001f && fabsf(command.pitch-3.9f)<.0001f);
            CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
            CHECK(driving.vx==10*(60+i) && shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
        }
        return 0;
    }
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
