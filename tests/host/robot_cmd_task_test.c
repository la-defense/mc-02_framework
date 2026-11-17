#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "remote_control.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "vision_control.h"
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL:%d %s\n",__LINE__,#c); return 1; } } while(0)
UART_HandleTypeDef huart5, huart9;
CAN_HandleTypeDef hcan1;
static RC_ctrl_t remote[2];
static uint8_t remote_online=1;
static uint32_t pc_sample_sequence=1u; // 测试替身只有显式递增时才交付新鼠标位移。
static uint32_t consumed_pc_sequence; // 每份替身位移最多交付一次。
static uint8_t fixed_source_enabled; // 双板期限场景固定来源时间。
static uint32_t fixed_source_ms;
static uint32_t tick=100;
static Vision_Recv_s wire;
static Vision_Status_t status;
static unsigned snapshots;
static uint8_t inject_short_fault; // 在命令选择期间注入已经恢复的短故障。
uint32_t HAL_GetTick(void) { return tick; }
const RC_ctrl_t *RemoteControlInit(PlatformUartPort_t port) { (void)port; return remote; }
uint8_t RemoteControlIsOnline(void) { return remote_online; }
void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot)
{ snapshot->input=remote[0]; snapshot->remote_online=remote_online; snapshot->pc_valid=remote_online; snapshot->pc_sample_sequence=pc_sample_sequence; snapshot->radio_source_time_ms=fixed_source_enabled ? fixed_source_ms : tick; snapshot->pc_source_time_ms=fixed_source_enabled ? fixed_source_ms : tick; }
RemoteMouseStatus_t RemoteControlConsumeMouse(uint32_t through, RemoteMouseBatch_t *batch)
{
    if (batch == NULL) return REMOTE_MOUSE_BAD_ARGUMENT;
    memset(batch, 0, sizeof(*batch));
    const uint32_t advance = through - consumed_pc_sequence; // 测试替身按来源序号去重并允许回绕。
    if (through == 0u || (consumed_pc_sequence != 0u && (advance == 0u || advance > INT32_MAX))) return REMOTE_MOUSE_EMPTY;
    consumed_pc_sequence = through;
    if (remote[0].mouse.x == 0 && remote[0].mouse.y == 0) return REMOTE_MOUSE_EMPTY;
    batch->x = remote[0].mouse.x; batch->y = remote[0].mouse.y;
    batch->oldest_source_ms = tick; batch->count = 1u;
    return REMOTE_MOUSE_OK;
}
Vision_Recv_s *VisionInit(PlatformUartPort_t h) { (void)h; return &wire; }
void VisionSend(void) {}
void VisionGetSnapshot(Vision_Recv_s *frame,Vision_Status_t *link)
{
    *frame=wire; *link=status; ++snapshots; // 提供本轮视觉数据。
    if (inject_short_fault) // 验证命令读取和发布之间发生的短故障。
    {
        inject_short_fault = 0u; // 只在一个周期注入。
        RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 记录故障禁止。
        RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF); // 模拟同一轮中已恢复。
        RobotSafetyUpdate(); // 旧状态名称可能重新回到 READY。
    }
}
/* Subscribe to production messages rather than inspecting task-private state. */
static int shoot_cycle(MessagePublisher_t *feedback, MessageSubscriber_t *gimbal, MessageSubscriber_t *shoot,
                       Gimbal_Upload_Data_s *pose, shoot_mode_e enabled,
                       friction_mode_e friction, loader_mode_e load, float rate, int vision)
{
    Gimbal_Ctrl_Cmd_s command; Shoot_Ctrl_Cmd_s shooting;
    unsigned before=snapshots;
    MessagePublishNow(feedback, pose); RobotCMDTask();
    CHECK(snapshots==before+1);
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(shooting.shoot_mode==enabled && shooting.friction_mode==friction);
    CHECK(shooting.load_mode==load && shooting.shoot_rate==rate);
    if (vision) CHECK(command.yaw_vel>0 && command.pitch_acc<0);
    else CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    return 0;
}

static int shoot_switches(MessagePublisher_t *feedback, MessageSubscriber_t *gimbal, MessageSubscriber_t *shoot,
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
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,1); // 视觉只能辅助瞄准，人工未请求发射。
    /* 人工改变拨轮时，发射跟随人工请求。 */
    remote[0].rc.dial=-600;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,1);
    wire.mode=1;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,1);
    wire.mode=0;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,0);
    remote[0].rc.switch_left=RC_SW_UP;
    remote[0].key_count[KEY_PRESS][Key_E]=0;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,0);
    wire.mode=2;
    CYCLE(SHOOT_OFF,FRICTION_OFF,LOAD_STOP,8,1);
    /* 视觉辅助期间，E/F 切换立即改变人工发射请求。 */
    remote[0].key_count[KEY_PRESS][Key_E]=2;
    remote[0].key_count[KEY_PRESS][Key_F]=1;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,1);
    wire.mode=0;
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
    /* Unsigned frame age: 50ms allowed, 51ms revoked, including wrap. */
    for (unsigned wrap=0;wrap<2;++wrap) {
        tick=wrap ? UINT32_MAX-20 : 200;
        wire.mode=2; status.last_rx_ms=tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,1);
        tick+=50;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,1);
        ++tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,0);
    }
    /* Reject each non-finite field after a valid takeover, not just pitch. */
    for (unsigned field=0;field<6;++field) for (unsigned inf=0;inf<2;++inf) {
        wire=(Vision_Recv_s){.mode=2,.yaw=.25f,.pitch=.1f,.yaw_vel=.2f,.pitch_acc=-.3f};
        status.last_rx_ms=tick;
        CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,1);
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
    CYCLE(SHOOT_ON,FRICTION_ON,LOAD_3_BULLET,8,1);
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

// 视觉帧有效时，人工模式的击发请求仍必须由操作者决定。
static int shoot_manual_authority(MessagePublisher_t *feedback, MessageSubscriber_t *gimbal,
                                  MessageSubscriber_t *shoot, Gimbal_Upload_Data_s *pose)
{
    remote[0].rc.switch_left = RC_SW_DOWN; // 选择现有人工遥控入口。
    remote[0].rc.dial = 0; // 人工没有请求发射。
    wire = (Vision_Recv_s){.mode=2, .yaw=.25f, .pitch=.1f, .yaw_vel=.2f, .pitch_acc=-.3f}; // 视觉请求连续发射。
    status.online = 1; // 视觉通信在线。
    status.last_rx_ms = tick; // 提供有效来源时刻。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_OFF, FRICTION_OFF, LOAD_STOP, 8, 1)); // 视觉不能自行开启发射。
    remote[0].rc.dial = -600; // 操作者请求连续发射。
    wire.mode = 1; // 视觉此时只要求停止供弹。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_ON, FRICTION_ON, LOAD_BURSTFIRE, 8, 1)); // 供弹仍按人工请求。
    remote[0].rc.switch_left = RC_SW_MID; // 既没有人工控制，也没有正式自主许可。
    wire.mode = 2; // 即使视觉继续请求发射也不能取得权力。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_OFF, FRICTION_OFF, LOAD_STOP, 0, 1)); // 缺少明确自主许可时关闭发射。
    return 0;
}

// 生产命令入口只有收到安全管理的正式自主击发许可时才采用视觉击发。
static int shoot_formal_authority(MessagePublisher_t *feedback, MessageSubscriber_t *gimbal,
                                  MessageSubscriber_t *shoot, Gimbal_Upload_Data_s *pose)
{
    RobotSafetyInit(); // 从新的安全状态开始，避免沿用主测试的人工使能。
    RobotSafetySetRcOnline(1u); // 本场景遥控在线，但自主许可不依赖它。
    RobotSafetySetImuValid(1u); // 测试核心姿态有效。
    RobotSafetySetCalibValid(1u); // 测试标定有效。
    RobotSafetySetTaskHealthy(1u); // 测试核心任务正常。
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 1u, 0u}; // 测试车型允许正式自主。
    CHECK(RobotSafetyConfigureControl(&policy)); // 启动时明确配置策略。
    RobotSafetyUpdate(); // 无比赛阶段时先保持禁止。
    ++tick; // 新来源出现在启动检查之后。
    RobotSafetyControlInputs_t input = {0}; // 只填写本场景提供的来源。
    input.mode = ROBOT_CONTROL_AUTONOMOUS; // 模拟命令仲裁者已选择正式自主。
    input.aim = (RobotControlLease_t){tick, 50u, 1u}; // 有效视觉瞄准。
    input.match = (RobotControlLease_t){tick, 100u, 1u}; // 有效比赛阶段。
    for (unsigned i = 0u; i < 3u; ++i) input.feedback[i] = (RobotControlLease_t){tick, 100u, 1u}; // 三组测试反馈均有效。
    input.feedback_valid_functions = ROBOT_CONTROL_ALL; // 声明测试机构已具备必需反馈。
    input.match_running = 1u; // 测试比赛阶段允许运行。
    input.vision_fire_requested = 1u; // 视觉提出击发请求。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 提交同一轮来源事实。
    RobotSafetyUpdate(); // 计算当前运行状态。
    CHECK(RobotSafetyIsReady()); // 确认核心和比赛条件允许自主运行。
    remote[0].rc.switch_left = RC_SW_MID; // 不选择人工命令。
    wire = (Vision_Recv_s){.mode=2, .yaw=.25f, .pitch=.1f, .yaw_vel=.2f, .pitch_acc=-.3f}; // 提供合法视觉击发帧。
    status.online = 1; // 测试视觉在线。
    status.last_rx_ms = tick; // 视觉帧与安全事实使用同一个来源时刻。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_ON, FRICTION_ON, LOAD_BURSTFIRE, 10, 1)); // 明确获准后才采用视觉击发。
    input.vision_fire_requested = 0u; // 视觉撤销击发请求。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 发布撤销后的事实。
    RobotSafetyUpdate(); // 更新安全状态。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_OFF, FRICTION_OFF, LOAD_STOP, 0, 1)); // 撤销许可后不能继续供弹。
    return 0;
}

// 分功能策略启用后，即使底盘仍有许可，发射局部禁止也必须覆盖命令。
static int shoot_local_permission(MessagePublisher_t *feedback, MessageSubscriber_t *gimbal,
                                  MessageSubscriber_t *shoot, Gimbal_Upload_Data_s *pose)
{
    RobotSafetyInit(); // 从新的安全状态开始。
    RobotSafetySetRcOnline(1u); // 测试遥控在线。
    RobotSafetySetImuValid(1u); // 测试姿态有效。
    RobotSafetySetCalibValid(1u); // 测试标定有效。
    RobotSafetySetTaskHealthy(1u); // 测试核心任务正常。
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u}; // 采用人工模式的分功能策略。
    CHECK(RobotSafetyConfigureControl(&policy)); // 在初始 SAFE 状态配置。
    RobotSafetyRequestEnable(); // 操作者请求使能。
    RobotSafetyUpdate(); // 等待新人工输入。
    ++tick; // 输入来源晚于使能起点。
    RobotSafetyControlInputs_t input = {0}; // 准备本轮测试事实。
    input.mode = ROBOT_CONTROL_MANUAL; // 指定人工模式。
    input.manual = (RobotControlLease_t){tick, 100u, 1u}; // 提交新鲜人工输入。
    for (unsigned i = 0u; i < 3u; ++i) input.feedback[i] = (RobotControlLease_t){tick, 100u, 1u}; // 三组测试反馈均有效。
    input.feedback_valid_functions = ROBOT_CONTROL_ALL; // 所有机构反馈均通过测试检查。
    input.manual_fire_requested = 1u; // 人工请求击发。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 先建立正常机构状态。
    RobotSafetyUpdate(); // 确认机构原先可用。
    ++tick; // 局部故障发生在正常状态之后。
    input.blocked_functions = ROBOT_CONTROL_SHOOTER; // 模拟仅发射机构故障。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 提交局部故障。
    RobotSafetyUpdate(); // 计算剩余功能许可。
    CHECK(RobotSafetyIsReady()); // 其他功能仍可运行。
    remote[0].rc.switch_left = RC_SW_DOWN; // 使用遥控人工命令。
    remote[0].rc.dial = -600; // 即使人工请求发射，也必须服从局部禁止。
    wire.mode = 0; // 本场景不依赖视觉。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_OFF, FRICTION_OFF, LOAD_STOP, 0, 0)); // 发射故障只关闭发射命令。
    ++tick; // 故障恢复发生在新的时刻。
    input.blocked_functions = 0u; // 解除局部故障。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 旧人工来源不能立即重新击发。
    RobotSafetyUpdate(); // 更新恢复屏障。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_OFF, FRICTION_OFF, LOAD_STOP, 0, 0)); // 旧命令仍被禁止。
    ++tick; // 操作者提供恢复后的新输入。
    input.manual.source_time_ms = tick; // 保留该新输入的原始时刻。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 提交恢复后的新请求。
    RobotSafetyUpdate(); // 重新计算发射许可。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_ON, FRICTION_ON, LOAD_BURSTFIRE, 8, 0)); // 新输入可重新接管。
    input.manual_fire_requested = 0u; // 人工撤销击发但保留机构运行许可。
    CHECK(RobotSafetyPublishControlInputs(&input)); // 提交新的击发事实。
    RobotSafetyUpdate(); // 重新计算击发许可。
    CHECK(!shoot_cycle(feedback, gimbal, shoot, pose, SHOOT_ON, FRICTION_ON, LOAD_STOP, 0, 0)); // 摩擦轮可运行，拨弹停止。
    return 0;
}
int main(int argc, char **argv)
{
    RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1);
    RobotSafetySetCalibValid(1); RobotSafetyRequestEnable(); RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    CHECK(RobotCMDInit());
#ifdef GIMBAL_BOARD
    if (argc > 1 && !strcmp(argv[1], "dual-freshness")) {
        MessageSubscriber_t *bridge=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
        MessagePublisher_t *feedback=MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK,ROBOT_TOPIC_ENDPOINT_GIMBAL);
        CHECK(bridge != NULL && feedback != NULL);
        Gimbal_Upload_Data_s pose={0}; Chassis_Ctrl_Cmd_s sent={0}; MessageFreshness_t time={0};
        remote[0].rc.switch_left=RC_SW_DOWN; remote[0].rc.switch_right=RC_SW_MID; remote[0].rc.rocker_r_=10;
        fixed_source_enabled=1u; fixed_source_ms=tick; // 遥控来源时间固定，任务仍持续运行。
        MessagePublishNow(feedback,&pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(bridge,&sent,sizeof(sent),tick,&time)==MESSAGE_OK && sent.vx==100.0f);
        CHECK(time.source_time_ms==fixed_source_ms && time.source_max_age_ms==0u); // 双板使用既有固定来源期限。
        tick+=50u; MessagePublishNow(feedback,&pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(bridge,&sent,sizeof(sent),tick,NULL)==MESSAGE_OK);
        ++tick; MessagePublishNow(feedback,&pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(bridge,&sent,sizeof(sent),tick,NULL)==MESSAGE_ERR_EMPTY); // 超过固定期限时撤销旧话题。
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "dual-board")) {
        MessageSubscriber_t *bridge=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
        CHECK(bridge != NULL);
        RobotCMDTask(); // 缺少云台反馈时仍向桥接话题发布停止命令。
        Chassis_Ctrl_Cmd_s sent={0};
        CHECK(MessageReadSnapshot(bridge,&sent,sizeof(sent),tick,NULL)==MESSAGE_OK);
        CHECK(sent.chassis_mode == CHASSIS_ZERO_FORCE);
        CHECK(sent.vx == 0.0f && sent.vy == 0.0f && sent.wz == 0.0f);
        puts("PASS: dual-board command publishes fail-safe topic output");
        return 0;
    }
#endif
    if (argc > 1 && !strcmp(argv[1], "gesture-zero")) {
        EstopRequest(ESTOP_REASON_MANUAL);
        remote[0].rc.switch_left=remote[0].rc.switch_right=RC_SW_MID;
        tick=0; RobotCMDTask();
        tick=999; RobotCMDTask(); CHECK(EstopIsLatched());
        tick=1000; RobotCMDTask(); CHECK(!EstopIsLatched());
        CHECK(RobotSafetyGetState()==ROBOT_SAFE);
        return 0;
    }
    MessagePublisher_t *feedback=MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessageSubscriber_t *gimbal=MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_COMMAND, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessageSubscriber_t *shoot=MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_SHOOT);
    Gimbal_Upload_Data_s pose={0}; pose.gimbal_imu_data.YawTotalAngle=12; pose.gimbal_imu_data.Pitch=3;
    Gimbal_Ctrl_Cmd_s command; Shoot_Ctrl_Cmd_s shooting;
    if (argc > 1 && !strcmp(argv[1], "mouse-sequence")) {
        remote[0].rc.switch_left=RC_SW_UP; remote[0].rc.switch_right=RC_SW_MID;
        remote[0].mouse.x=66; pc_sample_sequence=UINT32_MAX-1u; // 从回绕前的新样本开始。
        MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.yaw==13);
        pc_sample_sequence=UINT32_MAX; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.yaw==14);
        pc_sample_sequence=1u; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 来源会跳过保留的零。
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.yaw==15);
        pc_sample_sequence=UINT32_MAX; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 回绕前旧样本不能重放。
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.yaw==15);
        pc_sample_sequence=0u; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 零代表没有样本。
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.yaw==15);
        return 0;
    }
    remote[0].rc.switch_left=RC_SW_DOWN; remote[0].rc.switch_right=RC_SW_MID;
    if (argc > 1 && !strcmp(argv[1], "shoot-short-fault")) {
        MessageSubscriber_t *chassis = MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_CHASSIS);
        Chassis_Ctrl_Cmd_s driving = {0}; // 同时检查这一轮是否撤销底盘目标。
        RobotSafetySnapshot_t current = {0}; // 读取新发布命令所属的安全代际。
        remote[0].rc.dial = -600; // 人工请求连续供弹。
        inject_short_fault = 1u; // 故障发生在命令选取与最终发布之间。
        CHECK(!shoot_cycle(feedback,gimbal,shoot,&pose,SHOOT_OFF,FRICTION_OFF,LOAD_STOP,0,0)); // 整轮旧代际必须撤销。
        CHECK(MessageReadSnapshot(chassis, &driving, sizeof(driving), tick, NULL) == MESSAGE_OK);
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
        CHECK(driving.chassis_mode == CHASSIS_ZERO_FORCE && command.gimbal_mode == GIMBAL_ZERO_FORCE); // 短故障后运动也不能沿用旧一轮选择。
        ++tick; // 下一轮重新读取人工来源。
        CHECK(!shoot_cycle(feedback,gimbal,shoot,&pose,SHOOT_ON,FRICTION_ON,LOAD_BURSTFIRE,8,0)); // 新代际可重新计算。
        CHECK(RobotSafetyGetSnapshot(&current));
        CHECK(MessageReadSnapshot(chassis, &driving, sizeof(driving), tick, NULL) == MESSAGE_OK);
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
        CHECK(driving.safety_generation == current.control_generation && command.safety_generation == current.control_generation); // 新命令带着本轮代际交给下游。
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "shoot-manual-authority"))
        return shoot_manual_authority(feedback,gimbal,shoot,&pose);
    if (argc > 1 && !strcmp(argv[1], "shoot-formal-authority"))
        return shoot_formal_authority(feedback,gimbal,shoot,&pose);
    if (argc > 1 && !strcmp(argv[1], "shoot-local-permission"))
        return shoot_local_permission(feedback,gimbal,shoot,&pose);
    if (argc > 1 && !strcmp(argv[1], "shoot-switches"))
        return shoot_switches(feedback,gimbal,shoot,&pose);
    if (argc > 1 && !strcmp(argv[1], "faults")) {
        MessageSubscriber_t *chassis=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_CHASSIS);
        Chassis_Ctrl_Cmd_s driving;
        const Robot_Fault_e faults[]={ROBOT_FAULT_MOTOR_OFFLINE,ROBOT_FAULT_CAN_BUSOFF,ROBOT_FAULT_TILT};
        remote[0].rc.dial=-600;
        remote[0].rc.rocker_l_=100; remote[0].rc.rocker_l1=-100;
        remote[0].rc.rocker_r_=50; remote[0].rc.rocker_r1=-50;
        for (unsigned i=0;i<3;++i) {
            wire.mode=2; wire.yaw=.25f; wire.pitch=.1f;
            wire.yaw_vel=.2f; wire.yaw_acc=.3f; wire.pitch_vel=-.2f; wire.pitch_acc=-.3f;
            status.online=1; tick+=5; status.last_rx_ms=tick;
            MessagePublishNow(feedback, &pose); RobotCMDTask();
            CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(chassis, &driving, sizeof(driving), HAL_GetTick(), NULL) == MESSAGE_OK);
            CHECK(RobotSafetyIsReady() && command.gimbal_mode==GIMBAL_GYRO_MODE);
            CHECK(driving.vx!=0 && driving.vy!=0 && shooting.load_mode==LOAD_BURSTFIRE);
#if VISION_LIMITS_CONFIGURED
            CHECK(command.yaw_vel>0 && shooting.shoot_rate==8);
#else
            CHECK(command.yaw_vel==0 && shooting.shoot_rate==8);
#endif
            RobotFaultReport(faults[i]); tick+=5; status.last_rx_ms=tick;
            MessagePublishNow(feedback, &pose); RobotCMDTask();
            CHECK(RobotSafetyGetState()==ROBOT_FAULT);
            CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(chassis, &driving, sizeof(driving), HAL_GetTick(), NULL) == MESSAGE_OK);
            CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0);
            CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
            CHECK(driving.chassis_mode==CHASSIS_ZERO_FORCE && driving.vx==0 && driving.vy==0 && driving.wz==0 && driving.offset_angle==0);
            CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
            /* Recovery anchors to this cycle's pose and current manual inputs. */
            pose.gimbal_imu_data.YawTotalAngle=20+i; pose.gimbal_imu_data.Pitch=4;
            wire.mode=0; remote[0].rc.rocker_r_=60+i;
            RobotFaultClear(faults[i]); tick+=5;
            MessagePublishNow(feedback, &pose); RobotCMDTask();
            CHECK(RobotSafetyIsReady());
            CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(chassis, &driving, sizeof(driving), HAL_GetTick(), NULL) == MESSAGE_OK);
            CHECK(fabsf(command.yaw-(20.2f+i))<.0001f && fabsf(command.pitch-3.9f)<.0001f);
            CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
            CHECK(driving.vx==10*(60+i) && shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
        }
        return 0;
    }
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK); CHECK(command.gimbal_mode==GIMBAL_GYRO_MODE);
    remote[0].rc.switch_left=RC_SW_MID; tick+=5;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK); CHECK(MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
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
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(snapshots==before+1);
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.gimbal_mode==GIMBAL_GYRO_MODE && fabsf(command.yaw-14.323945f)<0.0001f);
    CHECK(fabsf(command.pitch-10.729578f)<0.0001f && command.yaw_vel>0 && command.pitch_acc<0);
    CHECK(shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
    tick+=10; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 只刷新姿态，不替视觉帧续期。
    MessageFreshness_t vision_time={0};
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), tick, &vision_time) == MESSAGE_OK);
    CHECK(vision_time.source_time_ms==status.last_rx_ms && vision_time.source_max_age_ms==VISION_CONTROL_MAX_AGE_MS && vision_time.published_time_ms==tick);
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), status.last_rx_ms+51u, NULL) == MESSAGE_ERR_EXPIRED);
    wire.mode=1; tick+=5; status.last_rx_ms=tick;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.yaw_vel>0 && shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
    CHECK(shooting.friction_mode==FRICTION_ON);
    wire.mode=0; tick+=5;
    remote[0].rc.rocker_l_=100; remote[0].rc.rocker_l1=-100;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(fabsf(command.yaw-12.2f)<0.0001f && fabsf(command.pitch-2.9f)<0.0001f);
    CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    CHECK(shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
    remote[0].rc.rocker_l_=remote[0].rc.rocker_l1=0;
    /* Each invalidation follows a real active vision cycle. */
    for (unsigned reason=0;reason<6;reason++) {
        status.online=1; wire.mode=2; wire.pitch=0.1f; tick+=5; status.last_rx_ms=tick;
        MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
        CHECK(command.yaw_vel>0 && shooting.shoot_rate==8);
        tick+=5;
        if (reason==0) status.online=0;
        if (reason==1) tick=status.last_rx_ms+51;
        if (reason==2) wire.pitch=1.0f;
        if (reason==3) wire.pitch=NAN;
        if (reason==4) wire.mode=0;
        if (reason==5) wire.mode=3;
        MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
        CHECK(command.yaw==12 && command.pitch==3 && command.gimbal_mode==GIMBAL_GYRO_MODE);
        CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
        CHECK(shooting.shoot_rate==8 && shooting.load_mode==LOAD_BURSTFIRE);
    }
    /* Missing/non-finite handoff feedback must not leave the old target active. */
    for (unsigned reason=0;reason<3;reason++) {
        wire.mode=2; wire.pitch=0.1f; status.online=1; tick+=5; status.last_rx_ms=tick;
        MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
        wire.mode=0; tick+=5;
        Gimbal_Upload_Data_s invalid=pose;
        if (reason==1) invalid.gimbal_imu_data.Pitch=NAN;
        if (reason==2) invalid.gimbal_imu_data.YawTotalAngle=INFINITY;
        if (reason) MessagePublishNow(feedback, &invalid);
        RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
        if (reason == 0u)
            CHECK(command.gimbal_mode==GIMBAL_GYRO_MODE && command.yaw==12 && command.pitch==3);
        else
            CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0);
        CHECK(command.yaw_vel==0 && command.pitch_acc==0);
        if (reason == 0u)
            CHECK(shooting.load_mode==LOAD_BURSTFIRE && shooting.shoot_rate==8);
        else
            CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF &&
                  shooting.load_mode==LOAD_STOP);
    }
    /* Keyboard mappings remain current after vision is revoked. */
    remote[0].rc.switch_left=RC_SW_UP;
    remote[0].key_count[KEY_PRESS][Key_E]=3; remote[0].key_count[KEY_PRESS][Key_F]=1;
    wire.mode=2; status.online=1; tick+=5; status.last_rx_ms=tick;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    wire.mode=0; remote[0].mouse.x=66; remote[0].mouse.y=-66; ++pc_sample_sequence; tick+=5;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.yaw==12 && command.pitch==3 && command.yaw_vel==0 && command.pitch_acc==0); // 视觉退出当轮的积压位移不跨模式重放。
    CHECK(shooting.shoot_rate==8 && shooting.load_mode==LOAD_BURSTFIRE);
    tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 读取同一鼠标样本不能再次移动目标。
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.yaw==12 && command.pitch==3);
    ++pc_sample_sequence; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 新样本才允许再次累加。
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.yaw==13 && command.pitch==2);
    wire.mode=2; ++pc_sample_sequence; tick+=5; status.last_rx_ms=tick; // 视觉控制期间也记录已观察的鼠标样本。
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    wire.mode=0; tick+=5; MessagePublishNow(feedback, &pose); RobotCMDTask(); // 回到手动时不能重放视觉期间的位移。
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.yaw==12 && command.pitch==3);
    /* No selected manual source after mode 2: complete output revocation. */
    wire.mode=2; tick+=5; status.last_rx_ms=tick;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    wire.mode=0; remote[0].rc.switch_left=RC_SW_MID; tick+=5;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.yaw_vel==0);
    CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
#else
    /* Actual default config refuses otherwise valid mode 2. */
    wire.mode=2; wire.yaw=0.25f; wire.pitch=0.1f; status.online=1; status.last_rx_ms=tick;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
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
        MessagePublishNow(feedback, &pose); RobotCMDTask();
        CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
        CHECK(!RobotSafetyIsReady());
        CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0 && command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
        CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF && shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
    }
    RobotSafetyInit(); RobotSafetySetRcOnline(1); RobotSafetySetImuValid(1); RobotSafetySetCalibValid(1);
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    remote_online=0; tick+=5;
    MessagePublishNow(feedback, &pose); RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && shooting.shoot_mode==SHOOT_OFF);
    /* Expired gimbal feedback fails closed and clears the manual handoff base. */
    tick += 51u;
    RobotCMDTask();
    CHECK(MessageReadSnapshot(gimbal, &command, sizeof(command), HAL_GetTick(), NULL) == MESSAGE_OK && MessageReadSnapshot(shoot, &shooting, sizeof(shooting), HAL_GetTick(), NULL) == MESSAGE_OK);
    CHECK(command.gimbal_mode==GIMBAL_ZERO_FORCE && command.yaw==0 && command.pitch==0);
    CHECK(command.yaw_vel==0 && command.yaw_acc==0 && command.pitch_vel==0 && command.pitch_acc==0);
    CHECK(shooting.shoot_mode==SHOOT_OFF && shooting.friction_mode==FRICTION_OFF &&
          shooting.load_mode==LOAD_STOP && shooting.shoot_rate==0);
    puts("PASS: production command task revokes outputs without a fresh feedback snapshot");
    return 0;
}
