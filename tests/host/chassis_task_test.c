#include "chassis.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "robot_def.h"
#include "config.h"
#include "referee_task.h"
#include "super_cap.h"
#include "general_def.h"
#ifdef CHASSIS_BOARD
#include "robot_safety.h"
#endif
#if defined(CHASSIS_BRIDGE_INTEGRATION)
#include "bsp_can.h" // 测试模型使用旧驱动类型注入接收帧。
#include "can_topic_link.h"
#include "chassis_topic_codec.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

static uint32_t test_tick;
static uint32_t tick_read_count;
static uint32_t advance_tick_on_read;
static uint32_t advance_tick_by_ms;
#if defined(CHASSIS_TEST_TRACK_CRITICAL)
unsigned chassis_test_critical_depth; // 平台替身记录临界区的进入与退出。
static uint8_t track_wheel_mutations;
static unsigned observed_wheel_mutations;
static unsigned unprotected_wheel_mutations;
#endif

static void NoteWheelMutation(void)
{
#if defined(CHASSIS_TEST_TRACK_CRITICAL)
    if (!track_wheel_mutations) return;
    ++observed_wheel_mutations;
    if (chassis_test_critical_depth == 0u)
        ++unprotected_wheel_mutations; // 模拟单轮操作退出自身保护后，派发任务可能看到只改了一部分的轮组。
#endif
}
uint32_t HAL_GetTick(void)
{
    tick_read_count++;
    if (advance_tick_on_read != 0u && tick_read_count == advance_tick_on_read)
    {
        test_tick += advance_tick_by_ms;
        advance_tick_on_read = 0u;
    }
    return test_tick;
}
CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1;
#define TEST_MECANUM_GEAR_RATIO 19.0f // 只供测试的电机轴与轮轴转速比。
static Chassis_Config_s chassis_configuration = {
    .wheel_type = MECANUM_WHEEL, // 默认覆盖麦轮的历史轮序和符号。
    .mecanum_geometry = {
        .half_base_m = 0.175f, // 测试半轴距，m。
        .half_track_m = 0.150f, // 测试半轮距，m。
        .radius_m = 0.060f, // 测试轮半径，m，计算中心取轮组几何中心。
    },
    .motion_limits = {
#if defined(CHASSIS_MOTION_LIMITS_CONFIGURED)
        .configured = CHASSIS_MOTION_LIMITS_CONFIGURED,
        .max_wz_deg_s = CHASSIS_MAX_WZ_DEG_S,
        .rotate_wz_deg_s = CHASSIS_ROTATE_WZ_DEG_S,
        .max_motor_speed_deg_s = CHASSIS_MAX_MOTOR_SPEED_DEG_S,
#else
        .configured = 0u,
#endif
    },
    .chassis_motor_config = {
        .dji_chassis_output_limit_raw = 60u, // 专用于验证四轮继承的测试报文限幅。
        .can_init_config = {.can_bus = PLATFORM_CAN_BUS_1},
        .controller_param_init_config = {.speed_PID = {.Kp = 12.0f}},
        .output_axis = {.gear_ratio = TEST_MECANUM_GEAR_RATIO}, // 公共参数不提供安装方向。
        .motor_type = M3508,
    },
    .wheel_motors = {
        [MOTOR_LF] = {1u, MOTOR_AXIS_DIRECTION_REVERSE}, // 左前测试电调 1，反向安装。
        [MOTOR_RF] = {2u, MOTOR_AXIS_DIRECTION_NORMAL}, // 右前测试电调 2，同向安装。
        [MOTOR_LB] = {4u, MOTOR_AXIS_DIRECTION_NORMAL}, // 左后测试电调 4，同向安装。
        [MOTOR_RB] = {3u, MOTOR_AXIS_DIRECTION_REVERSE}, // 右后测试电调 3，反向安装。
    },
};
const Chassis_Config_s *ChassisConfigFeed(void) { return &chassis_configuration; }
static DJIMotorInstance motors[4];
static Motor_Init_Config_s motor_configs[4];
static unsigned motor_count;
static unsigned rollback_count; // 保存测试期间已撤销的电机数。
static uint8_t rollback_failed; // 用于注入电机回退失败。
static unsigned fail_motor_attempt; // 非零时在指定电机申请处注入失败。
static unsigned motor_attempt; // 记录电机申请次数。
static uint8_t motor_rolled_back[4]; // 记录各电机是否已撤销。
static unsigned operation_order;
static uint8_t reject_motion_reference; // 模拟安全代际失效时电机拒绝目标。
static uint32_t observed_motion_generation; // 保存底盘交给电机的命令代际。
static uint32_t observed_motion_deadline_ms; // 保存底盘交给电机的原始截止点。
static uint32_t observed_direction_sequence; // 保存底盘交给电机的原方向来源。
static uint32_t observed_power_generation; // 保存交给轮电机的原供电区间。
static DJIChassisReferenceSourceCheck_t source_check; // 保存底盘启动时绑定的生产检查函数。
#if defined(MC02_VISION_PROTOCOL_V2)
static uint32_t referee_power_generation; // 模拟当前裁判供电区间，零代表撤权。

uint8_t RefereeBufferedPowerIsCurrent(uint32_t generation, uint32_t now_ms)
{
    return now_ms == test_tick && generation != 0u && generation == referee_power_generation;
}
#endif
static SuperCapInstance cap;
static int fail_cap, fail_motor;
static unsigned supercap_init_calls;
static unsigned supercap_rollback_calls; // 记录旧电容回退次数。
static uint8_t fail_cap_rollback; // 注入电容不能回退的情况。
#if defined(MC02_TEST_WRAP_CHASSIS_PAIR)
static uint8_t fail_topic_pair; // 注入底盘话题成对登记失败。
MessageStatus_t __real_MessageEndpointPairRegister(MessageTopicId_t, MessageTopicId_t, MessageEndpointId_t,
                                                     MessagePublisher_t **, MessageSubscriber_t **);
MessageStatus_t __wrap_MessageEndpointPairRegister(MessageTopicId_t pub, MessageTopicId_t sub,
    MessageEndpointId_t endpoint, MessagePublisher_t **publisher, MessageSubscriber_t **subscriber)
{
    if (fail_topic_pair) return MESSAGE_ERR_CAPACITY; // 测试中拒绝话题申请。
    return __real_MessageEndpointPairRegister(pub, sub, endpoint, publisher, subscriber); // 普通路径使用生产实现。
}
#endif
#ifdef CHASSIS_BOARD
static attitude_t chassis_attitude;
attitude_t *INS_GetStorage(void) { return &chassis_attitude; }
#endif

DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    ++motor_attempt; // 失败的申请也要计数。
    if (motor_attempt == fail_motor_attempt) return NULL; // 注入指定位置的注册失败。
    if (fail_motor) return NULL;
    if (motor_count >= 4)
        return NULL;
    motor_configs[motor_count] = *config;
    return &motors[motor_count++];
}
uint8_t DJIMotorRollbackStartup(DJIMotorInstance *motor)
{
    if (rollback_failed) return 0u; // 注入不能安全释放的情况。
    for (unsigned i = 0u; i < 4u; ++i)
        if (motor == &motors[i] && !motor_rolled_back[i])
        {
            motor_rolled_back[i] = 1u; // 标记指定实例被撤销。
            ++rollback_count; // 统计全部成功撤销的电机。
            return 1u;
        }
    return 0u; // 重复或未知实例不能视为成功。
}
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{ if (motor) { motor->ref = ref; motor->set_ref_order = ++operation_order; NoteWheelMutation(); } }
uint8_t DJIMotorBindChassisReferenceSource(DJIChassisReferenceSourceCheck_t check)
{
    source_check = check; // 后续由用例直接核对驱动能读取到的来源状态。
    return check != NULL;
}
uint8_t DJIMotorSetChassisRefUntilWithSource(DJIMotorInstance *motor, float ref, uint32_t generation,
                                            uint32_t deadline_ms, uint32_t source_sequence)
{
    observed_motion_generation = generation; // 核对底盘是否传递原命令的代际。
    observed_motion_deadline_ms = deadline_ms; // 测试期限不会在任务消费时刷新。
    observed_direction_sequence = source_sequence; // 保存这一轮真正提交的方向序号。
    observed_power_generation = 0u; // 方向提交没有裁判依赖。
    if (motor == NULL) return 0u; // 无实例时不能声称已写入。
    if (reject_motion_reference && motor == &motors[1]) return 0u; // 第一台轮电机已写入后，拒绝第二台。
    DJIMotorSetRef(motor, ref); // 保留已有的参考和调用顺序记录。
    return 1u;
}
uint8_t DJIMotorSetChassisRefUntilWithPower(DJIMotorInstance *motor, float ref, uint32_t generation,
                                           uint32_t deadline_ms, uint32_t power_generation)
{
    uint8_t accepted = DJIMotorSetChassisRefUntilWithSource(motor, ref, generation, deadline_ms, 0u);
    observed_power_generation = power_generation; // 复用参考记录，仅改变来源类型。
    return accepted;
}
void DJIMotorStop(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 0; ++motor->stop_count; NoteWheelMutation(); } }
void DJIMotorEnable(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 1; ++motor->enable_count; motor->enable_order = ++operation_order; NoteWheelMutation(); } }
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{ if (summary) { memset(summary, 0, sizeof(*summary)); if (motor) summary->online = motor->feedback_online; } }
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(DJIMotorInstance *motor, uint32_t now_ms,
                                                     Motor_Feedback_Snapshot_s *snapshot)
{
    (void)now_ms;
    if (motor == NULL || snapshot == NULL) return MOTOR_FEEDBACK_BAD_ARGUMENT;
    snapshot->speed.valid = motor->feedback_speed_valid;
    snapshot->speed.unit = motor->feedback_wrong_unit ? 0u : MOTOR_UNIT_RAD_S; // 注入错误单位时估速必须拒绝。
    snapshot->speed.reference = MOTOR_REFERENCE_OUTPUT_AXIS; // 该替身提供减速器输出轴数据。
    snapshot->speed.value = motor->feedback_speed_rad_s;
    snapshot->speed.source_time_ms = motor->feedback_source_time_ms;
    snapshot->speed.sequence = motor->feedback_sequence;
    snapshot->continuity_generation = motor->feedback_continuity_generation; // 模拟已恢复的有效轮速仍携带间断记录。
    snapshot->current.valid = motor->feedback_current_valid;
    if (motor->feedback_online == 0u) return MOTOR_FEEDBACK_NOT_READY;
    if (motor->feedback_age_ms > 20u) return MOTOR_FEEDBACK_STALE;
    return MOTOR_FEEDBACK_OK;
}
uint8_t DJIMotorSpeedFeedbackIsContinuous(const DJIMotorInstance *motor,
                                          uint32_t generation, uint32_t now_ms)
{
    return motor != NULL && motor->feedback_online && motor->feedback_speed_valid &&
        motor->feedback_continuity_generation == generation && // 保留测试注入的短暂间断。
        motor->feedback_age_ms <= DJI_FEEDBACK_MAX_AGE_MS &&
        now_ms - motor->feedback_source_time_ms <= DJI_FEEDBACK_MAX_AGE_MS; // 使用轮速的原始时刻核对期限。
}
SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *config) { (void)config; ++supercap_init_calls; return fail_cap ? NULL : &cap; }
uint8_t SuperCapRollbackStartup(SuperCapInstance *instance)
{
    if (instance != &cap) return 0u; // 测试替身只允许释放自身实例。
    ++supercap_rollback_calls; // 记录底盘尝试撤销电容实例。
    return (uint8_t)!fail_cap_rollback; // 测试可模拟底层拒绝回退。
}
RefereeStatus_t RefereeInit(PlatformUartPort_t port, const RefereeConfig_t *config)
{ (void)port; (void)config; return REFEREE_OK; }
void MC02ChassisTestLogError(void) {}

#if defined(CHASSIS_BRIDGE_INTEGRATION)
static FDCAN_HandleTypeDef bridge_can_handle;
static CANInstance bridge_can;
static void (*bridge_receive)(CANInstance *);
FDCAN_HandleTypeDef *BoardCanResolve(PlatformCanBus_t bus)
{ return bus == PLATFORM_CAN_BUS_1 ? &bridge_can_handle : NULL; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    bridge_can.can_handle=config->can_handle;
    bridge_can.tx_id=config->tx_id;
    bridge_can.id=config->id;
    bridge_receive=config->can_module_callback;
    return &bridge_can;
}
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *handle,const void *source,
    const CAN_TxFrame_t *frames,size_t count,CAN_TxPriority_t priority,uint32_t deadline,uint32_t *id)
{ (void)handle; (void)source; (void)frames; (void)count; (void)priority; (void)deadline; (void)id; return CAN_TX_SUBMIT_NOT_READY; }
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *handle,const void *source,
    const CAN_TxFrame_t *frames,size_t count,CAN_TxPriority_t priority,uint32_t expiry,uint32_t *id)
{ (void)handle; (void)source; (void)frames; (void)count; (void)priority; (void)expiry; (void)id; return CAN_TX_SUBMIT_NOT_READY; }
uint8_t CANGetTxRequestState(uint32_t id,CAN_TxResult_t *result) { (void)id; (void)result; return 0u; }
uint8_t CANCancelTxRequest(uint32_t id) { (void)id; return 1u; }
static uint8_t bridge_map(void *context,uint32_t remote,uint32_t now,uint32_t *local,uint32_t *uncertainty)
{ (void)context; (void)now; *local=remote; *uncertainty=2u; return 1u; }

static int test_bridge_to_chassis(void)
{
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    for (unsigned i=0u;i<4u;++i) {
        motors[i].feedback_online=1u;
        motors[i].feedback_speed_valid=1u;
        motors[i].feedback_current_valid=1u;
    }
    MessagePublisher_t *publisher=MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    MessageSubscriber_t *reader=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CHASSIS);
    const CanTopicCodec_t decoder={ChassisCommandBridgeDecode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
    const PlatformCanEndpoint_t config={.can_bus=PLATFORM_CAN_BUS_1,.tx_id=0x311u,.rx_id=0x312u};
    CanTopicLink_t link={0};
    CHECK(publisher && reader && CanTopicLinkInit(&link,&config,NULL,publisher,NULL,&decoder,bridge_map,NULL,50u));
    Chassis_Ctrl_Cmd_s command={.vx=100.0f,.chassis_mode=CHASSIS_NO_FOLLOW};
    CanTopicMessage_t message={.topic_id=ROBOT_TOPIC_CHASSIS_COMMAND,.length=CHASSIS_COMMAND_WIRE_BYTES,.source_ms=10u};
    CHECK(ChassisCommandEncode(&command,message.payload,sizeof(message.payload))==CHASSIS_CODEC_OK);
    CanTopicFrame_t frames[CAN_TOPIC_MAX_FRAMES]; size_t count=0u;
    CHECK(CanTopicEncode(&message,1u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK && count==5u);
    for (size_t i=0u;i<count;++i) {
        test_tick=11u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        uint32_t id=0u; CanTopicBridgeStatus_t sync;
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=18u;
    CHECK(CanTopicEncode(&message,2u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=24u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        uint32_t id=0u; CanTopicBridgeStatus_t sync;
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);
    // 重复的完整帧组不能把原命令的来源时间变成更晚的时间。
    for (size_t i=0u;i<count;++i) {
        test_tick=29u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
    }
    MessageFreshness_t freshness; Chassis_Ctrl_Cmd_s output;
    CHECK(MessageReadSnapshot(reader,&output,sizeof(output),test_tick,&freshness)==MESSAGE_OK &&
          freshness.source_time_ms==16u && freshness.received_time_ms==24u);
    test_tick=67u;
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    uint32_t id=0u; CanTopicBridgeStatus_t sync;
    (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    message.source_ms=20u;
    CHECK(CanTopicEncode(&message,2u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    frames[count-1u].bytes[2]^=1u;
    for (size_t i=0u;i<count;++i) {
        test_tick=68u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    CHECK(MessageReadSnapshot(reader,&output,sizeof(output),test_tick,&freshness)==MESSAGE_ERR_EMPTY);
    message.source_ms=72u;
    CHECK(CanTopicEncode(&message,4u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=73u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    CHECK(MessageReadSnapshot(reader,&output,sizeof(output),test_tick,&freshness)==MESSAGE_OK &&
          freshness.source_time_ms==70u && freshness.received_time_ms==73u);
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=84u;
    CHECK(CanTopicEncode(&message,5u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=78u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);

    // 本地任一轮电机离线后停止；重新在线时旧话题序号不能直接恢复运动。
    motors[2].feedback_online=0u;
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    motors[2].feedback_online=1u;
    test_tick=90u;
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    // 故障恢复后才组装的分片仍可能来自恢复前；仅序号更新不足以重新授权。
    message.source_ms=88u;
    CHECK(CanTopicEncode(&message,6u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=91u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=96u;
    CHECK(CanTopicEncode(&message,7u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=96u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);

    // 短暂 IMU 故障即使在两个底盘周期之间恢复，也必须等待故障后产生的新命令。
    RobotSafetySetImuValid(0u);
    RobotSafetySetImuValid(1u);
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=104u;
    CHECK(CanTopicEncode(&message,8u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=103u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);
    // 急停请求及解除都发生在任务周期之间时同样不得复用旧序号。
    EstopRequest(ESTOP_REASON_MANUAL);
    CHECK(EstopClear());
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=113u;
    CHECK(CanTopicEncode(&message,9u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=108u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);
    motors[1].feedback_age_ms=21u;
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    motors[1].feedback_age_ms=0u;
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    message.source_ms=118u;
    CHECK(CanTopicEncode(&message,10u,frames,CAN_TOPIC_MAX_FRAMES,&count)==CAN_TOPIC_OK);
    for (size_t i=0u;i<count;++i) {
        test_tick=113u+(uint32_t)i; bridge_can.rx_timestamp_ms=test_tick;
        bridge_can.rx_len=frames[i].length; memcpy(bridge_can.rx_buff,frames[i].bytes,frames[i].length);
        bridge_receive(&bridge_can);
        (void)CanTopicLinkService(&link,test_tick,&id,&sync);
    }
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled && motors[i].ref!=0.0f);
    return 0;
}
#endif

// 仅在软件测试使用尺寸已知的切向轮组。
static int TestMecanumVelocity(void)
{
    CHECK(ChassisInit() == 1u); // 沿用有 19:1 减速比的默认麦轮配置。
    ChassisVelocityFeedback_t velocity = {0};
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 0u); // 注册电机不等于收到实际轮速。

    const struct {
        float wheel_edge_m_s[CHASSIS_WHEEL_COUNT]; // 左前、右前、左后、右后电机槽位的有符号轮缘速度。
        ChassisTwist_t expected; // 以底盘前、左和绕上轴为正。
    } cases[] = {
        {{1.0f, 1.0f, -1.0f, -1.0f}, {1.0f, 0.0f, 0.0f}}, // 前进 1 m/s。
        {{-1.0f, 1.0f, -1.0f, 1.0f}, {0.0f, 1.0f, 0.0f}}, // 左移 1 m/s。
        {{0.325f, 0.325f, 0.325f, 0.325f}, {0.0f, 0.0f, 1.0f}}, // 以两段半尺寸之和 0.325 m 换算左转 1 rad/s。
        {{0.625f, 1.225f, -0.575f, 0.025f}, {0.6f, 0.3f, 1.0f}}, // 混合三轴，轮速各异，用来查出轮位互换。
    };
    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); ++c)
    {
        test_tick = 100u + c; // 每组样本使用不同的接收时刻。
        for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            motors[i].feedback_online = 1u; // 电调已收到反馈。
            motors[i].feedback_speed_valid = 1u; // 反馈是驱动换算后的输出轴转速。
            motors[i].feedback_speed_rad_s = cases[c].wheel_edge_m_s[i] / 0.060f; // 手工给出轮缘速度，不调用目标计算生成期望。
            motors[i].feedback_source_time_ms = test_tick; // 记录本轮各电机原始采样时间。
        }
        motors[MOTOR_RF].feedback_source_time_ms = test_tick - 2u; // 右前轮最旧，决定整组反馈期限。
        ChassisTask(); // 不发布命令也应保留滑行时的测量。
        CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && velocity.source_time_ms == test_tick - 2u);
        CHECK(fabsf(velocity.twist.vx_m_s - cases[c].expected.vx_m_s) < 0.0001f);
        CHECK(fabsf(velocity.twist.vy_m_s - cases[c].expected.vy_m_s) < 0.0001f);
        CHECK(fabsf(velocity.twist.wz_rad_s - cases[c].expected.wz_rad_s) < 0.0001f);
    }
    ChassisOdometryFeedback_t pose;
    CHECK(Chassis_GetOdometry(&pose) == 0u); // 麦轮仍只报告速度。
    test_tick += 18u; // 最早轮速刚好达到允许的 20 ms。
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u);
    ++test_tick; // 任务尚未刷新时，读者也必须拒绝过期数据。
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 0u && velocity.twist.vx_m_s == 0.0f);
    motors[MOTOR_RF].feedback_source_time_ms = test_tick; // 其余三轮仍在有效期内。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u);
    motors[MOTOR_LB].feedback_wrong_unit = 1u; // 一轮的单位错误会撤销整组速度。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 0u);
    return 0;
}

static void SelectOmniFixture(void)
{
    const float u = 0.70710678118f;
    chassis_configuration.wheel_type = OMNI_WHEEL;
    chassis_configuration.mecanum_geometry = (MecanumGeometry_t){0}; // 全向轮配置无须填写麦轮参数。
    chassis_configuration.omni_geometry = (OmniGeometry_t){.wheels = {
        {0.2f, 0.2f, -u, u, 0.1f}, // 左前轮位置、单位滚动方向与轮径。
        {0.2f, -0.2f, u, u, 0.1f}, // 右前轮。
        {-0.2f, 0.2f, -u, -u, 0.1f}, // 左后轮。
        {-0.2f, -0.2f, u, -u, 0.1f}, // 右后轮。
    }};
}

// 用已知的切向轮组测试生产任务发布速度及按原始时间失效。
static int TestOmniVelocity(void)
{
    SelectOmniFixture();
    chassis_configuration.omni_geometry.wheels[0].radius_m = 0.0f; // 唯一几何无效时，实际启动入口不得登记电机。
    CHECK(ChassisInit() == 0u && motor_attempt == 0u);
    SelectOmniFixture();
    const float u = 0.70710678118f;
    chassis_configuration.chassis_motor_config.output_axis.gear_ratio = 38.0f; // 仅测试，用较高减速比触发轮速上限。
    CHECK(ChassisInit() == 1u);
    ChassisVelocityFeedback_t sample = {0};
    CHECK(Chassis_GetVelocityFeedback(&sample) == 0u); // 启动本身不产生测量。

    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    const Chassis_Ctrl_Cmd_s command = {.vx = 1000.0f, .chassis_mode = CHASSIS_NO_FOLLOW}; // 本体前进 1 m/s。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask(); // 先检查发布给电机的输出轴速度，再把该速度作为理想反馈。
    const float output_limit_rad_s = chassis_configuration.motion_limits.max_motor_speed_deg_s * DEGREE_2_RAD / 38.0f;
    for (unsigned i = 0u; i < 4u; ++i)
    {
        CHECK(motors[i].enabled && motor_configs[i].output_axis.gear_ratio == 38.0f); // 驱动和限速使用同一配置。
        const float expected = i % 2u == 0u ? -output_limit_rad_s : output_limit_rad_s; // 左右轮滚动投影相反。
        CHECK(fabsf(motors[i].ref - expected) < 0.0001f); // 轮轴目标保留几何的输出轴方向。
        motors[i].feedback_online = motors[i].feedback_speed_valid = 1u; // 模拟驱动正常跟随目标。
        motors[i].feedback_speed_rad_s = motors[i].ref; // 反馈与目标都是输出轴 rad/s。
    }
    ChassisTask(); // 真实估速入口应恢复前进方向，而非把目标解释为后退。
    CHECK(Chassis_GetVelocityFeedback(&sample) == 1u && sample.twist.vx_m_s > 0.0f && sample.twist.vx_m_s < 1.0f);
    CHECK(MessagePublisherInvalidate(commands) == MESSAGE_OK); // 后面的独立反馈检查不使用运动许可。
    test_tick = 4u; // 四份反馈横跨毫秒计数回绕。
    for (unsigned i = 0u; i < 4u; ++i)
    {
        motors[i].feedback_online = 1u;
        motors[i].feedback_speed_valid = 1u;
        motors[i].feedback_speed_rad_s = i % 2u == 0u ? -10.0f * u : 10.0f * u; // 模拟底盘向前 1 m/s。
        motors[i].feedback_source_time_ms = test_tick; // 三轮较新。
    }
    motors[2].feedback_source_time_ms = UINT32_MAX - 4u; // 左后轮最早，重新估计也要保留此时刻。
    ChassisTask(); // 无运动许可时仍读取现有反馈。
    CHECK(Chassis_GetVelocityFeedback(&sample) == 1u);
    CHECK(sample.source_time_ms == UINT32_MAX - 4u && fabsf(sample.twist.vx_m_s - 1.0f) < 0.0001f);
    CHECK(fabsf(sample.twist.vy_m_s) < 0.0001f && fabsf(sample.twist.wz_rad_s) < 0.0001f);
    ChassisOdometryFeedback_t pose;
    CHECK(Chassis_GetOdometry(&pose) == 0u); // 默认关闭位姿，速度仍有效。
    test_tick = 15u; // 距最早反馈正好 20 ms，仍有效。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&sample) == 1u && sample.source_time_ms == UINT32_MAX - 4u);
    test_tick = 16u; // 发布者未再次运行，读者也须让样本过期。
    CHECK(Chassis_GetVelocityFeedback(&sample) == 0u && sample.twist.vx_m_s == 0.0f);
    ChassisTask(); // 即便替身仍报在线，估计也要检查原始时刻。
    CHECK(Chassis_GetVelocityFeedback(&sample) == 0u);

    motors[2].feedback_source_time_ms = test_tick;
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&sample) == 1u);
    motors[3].feedback_wrong_unit = 1u; // 一轮的数据不能借用其余三轮的有效性。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&sample) == 0u);
    motors[3].feedback_wrong_unit = 0u;
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&sample) == 1u);
    CHECK(ChassisReleaseStartupDevices() == 1u);
    CHECK(Chassis_GetVelocityFeedback(&sample) == 0u);
    return 0;
}

// 只刷新指定轮速，可在主机分别模拟同步和异步到达。
static void PublishForwardWheel(unsigned i, uint32_t time_ms)
{
    const float u = 0.70710678118f;
    motors[i].feedback_online = 1u; // 该轮反馈在线。
    motors[i].feedback_speed_valid = 1u; // 替身按输出轴 rad/s 交付。
    motors[i].feedback_speed_rad_s = i % 2u == 0u ? -10.0f * u : 10.0f * u; // 四轮等方向投影合成前进 1 m/s。
    motors[i].feedback_source_time_ms = time_ms; // 采样时间独立于执行任务的时刻。
    if (++motors[i].feedback_sequence == 0u) ++motors[i].feedback_sequence; // 同驱动一样跳过保留的零序号。
}

static void PublishForwardWheels(uint32_t time_ms)
{
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
        PublishForwardWheel(i, time_ms); // 同一毫秒为四轮交付独立样本。
}

static int TestOmniOdometry(void)
{
    ChassisOdometryFeedback_t result;
    chassis_configuration.odometry_max_gap_ms = 5u; // 测试值，先验证麦轮不能启用本功能。
    CHECK(ChassisInit() == 0u && motor_attempt == 0u);
    SelectOmniFixture();
    CHECK(ChassisInit() == 1u && Chassis_GetOdometry(&result) == 0u);

    test_tick = UINT32_MAX - 2u; // 从回绕前开始，下一样本的时间为零。
    PublishForwardWheels(test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.pose.x_m == 0.0f);
    const uint32_t first_segment = result.segment;
    test_tick = 0u;
    PublishForwardWheels(test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.source_time_ms == 0u && result.segment == first_segment);
    CHECK(fabsf(result.pose.x_m - 0.003f) < 0.00001f);

    test_tick = 2u;
    PublishForwardWheels(test_tick);
    motors[2].feedback_source_time_ms = 0u; // 其余轮速更新时，最早时刻没有前进。
    motors[1].feedback_speed_rad_s *= 2.0f; // 新旧混合速度也不应覆盖积分起点。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.source_time_ms == 0u);
    CHECK(fabsf(result.pose.x_m - 0.003f) < 0.00001f);
    test_tick = 5u;
    PublishForwardWheels(test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.source_time_ms == 5u && result.segment == first_segment);
    CHECK(fabsf(result.pose.x_m - 0.008f) < 0.00001f);

    test_tick = 11u; // 全部轮速仍在线，但相隔 6 ms 超过配置的 5 ms。
    PublishForwardWheels(test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.segment == first_segment + 1u && result.pose.x_m == 0.0f);
    motors[0].feedback_source_time_ms = 10u; // 仍在有效期的旧帧不能让积分时刻倒退。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 0u && result.pose.x_m == 0.0f && result.segment == 0u);
    PublishForwardWheels(test_tick); // 同一时刻的样本不能恢复已中断的积分。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 0u);
    PublishForwardWheels(++test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.segment == first_segment + 2u);
    motors[1].feedback_speed_valid = 0u; // 任一轮失效立即撤销位姿。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 0u);
    PublishForwardWheels(++test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.segment == first_segment + 3u);

    test_tick = 33u; // 发布者未再运行，13 ms 取得的样本仍在最后一个有效毫秒。
    CHECK(Chassis_GetOdometry(&result) == 1u);
    ++test_tick;
    CHECK(Chassis_GetOdometry(&result) == 0u && result.source_time_ms == 0u);
    PublishForwardWheels(18u); // 有效的延迟样本恰好相隔 5 ms，也不得接回失效的段。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&result) == 1u && result.segment == first_segment + 4u && result.pose.x_m == 0.0f);
    CHECK(ChassisReleaseStartupDevices() == 1u && Chassis_GetOdometry(&result) == 0u);
    return 0;
}

static int TestOmniOdometryBatch(void)
{
    SelectOmniFixture();
    chassis_configuration.odometry_max_gap_ms = 15u; // 主机用值，覆盖异步到达和倒退场景。
    CHECK(ChassisInit() == 1u);
    ChassisOdometryFeedback_t pose;
    ChassisVelocityFeedback_t velocity;
    test_tick = 12u;
    motors[0].feedback_sequence = UINT32_MAX - 1u; // 最早轮的首样本为最后一个非零序号。
    PublishForwardWheel(0u, 10u);
    for (unsigned i = 1u; i < CHASSIS_WHEEL_COUNT; ++i)
        PublishForwardWheel(i, 12u);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.source_time_ms == 10u);
    const uint32_t segment = pose.segment;

    test_tick = 15u;
    PublishForwardWheel(0u, 15u); // 只更新最早轮，序号回绕且四轮估速时刻已经前进。
    ChassisTask();
    const uint32_t before_read = tick_read_count;
    CHECK(Chassis_GetMotionFeedback(&velocity, &pose) == 3u);
    CHECK(tick_read_count == before_read + 1u); // 两组反馈在同一时刻复核。
    CHECK(velocity.source_time_ms == 12u && pose.source_time_ms == 10u);
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && velocity.source_time_ms == 12u);
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.source_time_ms == 10u && pose.pose.x_m == 0.0f);
    test_tick = 17u;
    for (unsigned i = 1u; i < CHASSIS_WHEEL_COUNT; ++i)
        PublishForwardWheel(i, 17u); // 等齐后才消耗四轮身份，最早新反馈仍在 15 ms。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.source_time_ms == 15u && pose.segment == segment);
    CHECK(fabsf(pose.pose.x_m - 0.005f) < 0.00001f);

    test_tick = 19u;
    PublishForwardWheels(19u);
    motors[1].feedback_source_time_ms = 16u; // 最早时刻仍前进，但单轮时间倒退。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && velocity.source_time_ms == 16u);
    CHECK(Chassis_GetOdometry(&pose) == 0u);
    test_tick = 22u;
    PublishForwardWheels(test_tick); // 确认全部样本刷新后从新段起算。
    motors[0].feedback_source_time_ms = 20u; // 留出两毫秒的轮速到达差。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.segment == segment + 1u && pose.pose.x_m == 0.0f);

    test_tick = 41u; // 上次位姿已过期，但只刷新最早轮后，当前四轮仍可估速。
    PublishForwardWheel(0u, test_tick);
    ChassisTask();
    CHECK(Chassis_GetMotionFeedback(&velocity, &pose) == 1u && pose.segment == 0u);
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && Chassis_GetOdometry(&pose) == 0u);
    test_tick = 42u;
    for (unsigned i = 1u; i < CHASSIS_WHEEL_COUNT; ++i)
        PublishForwardWheel(i, test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.segment == segment + 2u && pose.pose.x_m == 0.0f);

    test_tick = 44u;
    PublishForwardWheels(test_tick); // 先在当前段积出非零位移。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.segment == segment + 2u && pose.pose.x_m > 0.0f);
    test_tick = 45u;
    PublishForwardWheel(3u, test_tick); // 同一连续区间的新样本不撤销已发布的估计。
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && Chassis_GetOdometry(&pose) == 1u);
    motors[2].feedback_speed_valid = 0u; // 在两次底盘任务之间使一轮反馈失效。
    ++motors[2].feedback_continuity_generation; // 驱动记录这次间断。
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 0u && velocity.twist.vx_m_s == 0.0f);
    CHECK(Chassis_GetOdometry(&pose) == 0u && pose.segment == 0u);
    test_tick = 46u;
    PublishForwardWheel(2u, test_tick); // 同一轮已恢复，底盘尚未重新处理这次间断。
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 0u && Chassis_GetOdometry(&pose) == 0u);
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && Chassis_GetOdometry(&pose) == 0u); // 另外三轮未更新，撤销旧位姿。
    test_tick = 47u;
    PublishForwardWheels(test_tick);
    motors[0].feedback_source_time_ms = 45u; // 序号已更新，但这个样本来自断流轮恢复前。
    ChassisTask();
    CHECK(Chassis_GetVelocityFeedback(&velocity) == 1u && Chassis_GetOdometry(&pose) == 0u); // 防止从恢复前的时刻开始积分。
    test_tick = 48u;
    PublishForwardWheel(0u, test_tick); // 现在四轮都取得达到重建时刻的样本。
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.segment == segment + 3u && pose.pose.x_m == 0.0f);
    test_tick = 49u;
    PublishForwardWheels(test_tick);
    ChassisTask();
    CHECK(Chassis_GetOdometry(&pose) == 1u && pose.segment == segment + 3u);
    CHECK(fabsf(pose.pose.x_m - 0.002f) < 0.00001f); // 仅对新段内的两毫秒积分。
    return 0;
}

#if defined(ONE_BOARD)
#if defined(CHASSIS_TEST_TRACK_CRITICAL)
static int TestWheelGroupCommit(void)
{
    CHECK(ChassisInit() == 1u);
    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    CHECK(commands != NULL);
    test_tick = 100u;
    Chassis_Ctrl_Cmd_s command = {.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW}; // 本体目标没有外部方向或供电来源。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask(); // 先建立仍然启用的四轮旧参考。
    const float old_left = motors[0].ref;
    command.vx = -100.0f;
    test_tick++;
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    track_wheel_mutations = 1u;
    ChassisTask(); // 已启用轮位依次写入时，派发任务不能读取它们。
    CHECK(observed_wheel_mutations == 8u && unprotected_wheel_mutations == 0u && chassis_test_critical_depth == 0u);
    CHECK(motors[0].ref * old_left < 0.0f); // 这一轮确实以方向不同的参考替换旧值。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(motors[i].enabled && motors[i].ref != 0.0f);

    reject_motion_reference = 1u; // 左前先写入，右前提交失败后整个轮组必须一起撤销。
    observed_wheel_mutations = 0u;
    ChassisTask();
    CHECK(observed_wheel_mutations >= 8u && unprotected_wheel_mutations == 0u && chassis_test_critical_depth == 0u);
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);

    reject_motion_reference = 0u;
    ChassisTask(); // 重新建立四轮目标，再模拟调用任务的主动关断入口。
    observed_wheel_mutations = 0u;
    ChassisStopAndClear();
    CHECK(observed_wheel_mutations == 8u && unprotected_wheel_mutations == 0u && chassis_test_critical_depth == 0u);
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    return 0;
}
#endif

static int TestChassisDirectionRevocation(void)
{
    CHECK(ChassisInit() == 1u); // 使用实际底盘初始化和话题实现。
    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    MessagePublisher_t *feedback = MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessageSubscriber_t *direction_reader = MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_COMMAND);
    CHECK(commands != NULL && feedback != NULL && direction_reader != NULL);
    Gimbal_Upload_Data_s direction = {0}; // 零角度也有方向依赖，不能靠下发角度的正负识别。
    MessageFreshness_t direction_time;
    test_tick = 100u;
    CHECK(MessagePublishNow(feedback, &direction) == MESSAGE_OK);
    CHECK(MessageReadSnapshot(direction_reader, &direction, sizeof(direction), test_tick, &direction_time) == MESSAGE_OK);
    Chassis_Ctrl_Cmd_s command = {.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW,
        .local_direction_sequence = direction_time.sequence}; // 绑定本次人工方向的来源序号。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    CHECK(observed_direction_sequence == direction_time.sequence && source_check != NULL &&
          source_check(DJI_CHASSIS_SOURCE_DIRECTION, observed_direction_sequence, test_tick)); // 驱动拿到原序号及对应业务检查函数。

    ++test_tick;
    CHECK(MessagePublishNow(feedback, &direction) == MESSAGE_OK); // 连续来源的新样本不用等待命令任务重算。
    ChassisTask();
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    ++test_tick;
    CHECK(MessagePublisherInvalidate(feedback) == MESSAGE_OK);
    CHECK(!source_check(DJI_CHASSIS_SOURCE_DIRECTION, observed_direction_sequence, test_tick)); // 发送前检查可以独立于底盘任务发现撤销。
    ChassisTask(); // 底盘命令自己仍未到期，但其方向已明确撤销。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);

    ++test_tick;
    CHECK(MessagePublishAtBounded(feedback, &direction, test_tick, test_tick, test_tick,
        MESSAGE_CLOCK_LOCAL, 2u) == MESSAGE_OK);
    ChassisTask(); // 方向恢复也不能恢复撤销前发出的命令。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    CHECK(MessageReadSnapshot(direction_reader, &direction, sizeof(direction), test_tick, &direction_time) == MESSAGE_OK);
    command.local_direction_sequence = direction_time.sequence; // 新命令与恢复后的有效方向绑定。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    test_tick += 3u;
    ChassisTask(); // 方向期限到期，底盘命令本身仍在话题期限内。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);

    command.local_direction_sequence = 0u; // 本体参照在同一轮不依赖这份方向。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    return 0;
}
#if defined(MC02_VISION_PROTOCOL_V2)
static int TestChassisPowerRevocation(void)
{
    CHECK(ChassisInit() == 1u); // 使用生产任务及本地话题。
    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    CHECK(commands != NULL && source_check != NULL);
    test_tick = 100u;
    referee_power_generation = 7u; // 主机输入模拟首次有效的裁判区间。
    Chassis_Ctrl_Cmd_s command = {.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW, .local_power_generation = 7u};
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    CHECK(observed_power_generation == 7u && observed_direction_sequence == 0u);
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    referee_power_generation = 0u; // 命令未到期，但供电先撤销。
    ++test_tick;
    ChassisTask();
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    referee_power_generation = 8u;
    ChassisTask(); // 供电恢复时，旧命令仍绑定第 7 个区间。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    command.local_power_generation = referee_power_generation;
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    CHECK(observed_power_generation == 8u && motors[0].enabled && motors[0].ref != 0.0f);
    command.local_direction_sequence = 1u; // 不能把两种依赖误当成其中一种通过。
    CHECK(MessagePublishNow(commands, &command) == MESSAGE_OK);
    ChassisTask();
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    return 0;
}
#endif
#endif

int main(int argc, char **argv)
{
#if defined(ONE_BOARD)
#if defined(CHASSIS_TEST_TRACK_CRITICAL)
    if (argc == 2 && strcmp(argv[1], "wheel-group-commit") == 0) return TestWheelGroupCommit();
#endif
    if (argc == 2 && strcmp(argv[1], "direction-revocation") == 0) return TestChassisDirectionRevocation();
#if defined(MC02_VISION_PROTOCOL_V2)
    if (argc == 2 && strcmp(argv[1], "power-revocation") == 0) return TestChassisPowerRevocation();
#endif
#endif
    if (argc == 2 && strcmp(argv[1], "mecanum-velocity") == 0) return TestMecanumVelocity();
    if (argc == 2 && strcmp(argv[1], "velocity") == 0) return TestOmniVelocity();
    if (argc == 2 && strcmp(argv[1], "odometry") == 0) return TestOmniOdometry();
    if (argc == 2 && strcmp(argv[1], "odometry-batch") == 0) return TestOmniOdometryBatch();
    if (argc == 2 && strncmp(argv[1], "motor-at-", 9u) == 0)
        fail_motor_attempt = (unsigned)(argv[1][9] - '0'); // 注入指定轮电机注册失败。
    if (argc == 2 && strcmp(argv[1], "rollback-failure") == 0)
    {
        fail_motor_attempt = 2u; // 第二台电机失败前已有一台注册成功。
        rollback_failed = 1u; // 让已注册电机无法撤销。
    }
#if defined(MC02_TEST_WRAP_CHASSIS_PAIR)
    if (argc == 2 && strcmp(argv[1], "topic-failure") == 0) fail_topic_pair = 1u; // 在设备全部注册后拒绝话题。
    if (argc == 2 && strcmp(argv[1], "cap-rollback-failure") == 0)
    {
        fail_topic_pair = 1u; // 触发电容与轮电机回退。
        fail_cap_rollback = 1u; // 让电容回退失败。
    }
#endif
    if (argc == 2 && strcmp(argv[1], "cap-failure") == 0) fail_cap = 1;
    if (argc == 2 && strcmp(argv[1], "motor-failure") == 0) fail_motor = 1;
    uint8_t initialized = ChassisInit();
    if (fail_motor_attempt != 0u)
    {
        CHECK(initialized == 0u && ChassisInitRollbackFailed() == rollback_failed); // 失败及回退故障均需明确返回。
        if (!rollback_failed) CHECK(rollback_count == fail_motor_attempt - 1u); // 成功注册的轮电机均已撤销。
        CHECK(supercap_init_calls == 0u); // 电机申请失败后不应继续注册电容。
        return 0;
    }
#if defined(MC02_TEST_WRAP_CHASSIS_PAIR)
    if (fail_topic_pair)
    {
        CHECK(initialized == 0u && ChassisInitRollbackFailed() == fail_cap_rollback); // 回退故障必须上报。
        CHECK(rollback_count == 4u && supercap_rollback_calls == 1u); // 四台电机和电容全部撤销。
        return 0;
    }
#endif
#if defined(MC02_CONFIG_GENERATED)
    if (fail_cap) { CHECK(initialized == 0u && supercap_init_calls == 1u && rollback_count == 4u); return 0; }
#endif
    // 未实现普通功率回退前，电容注册或核心电机失败都保留原失败结果。
    if (fail_cap || fail_motor) { CHECK(initialized == 0u); return 0; }
    CHECK(initialized != 0u);
    CHECK(motor_count == 4);
    for (unsigned wheel = 0u; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
    {
        CHECK(motor_configs[wheel].can_init_config.tx_id == chassis_configuration.wheel_motors[wheel].device_id);
        CHECK(motor_configs[wheel].output_axis.direction == chassis_configuration.wheel_motors[wheel].direction);
        CHECK(motor_configs[wheel].dji_chassis_output_limit_raw ==
            chassis_configuration.chassis_motor_config.dji_chassis_output_limit_raw); // 四轮继承同一车型输出上限。
    }
    if (argc == 2 && strncmp(argv[1], "release-", 8u) == 0)
    {
        rollback_failed = (uint8_t)(strcmp(argv[1], "release-failure") == 0); // 初始化完成后再拒绝电机回退。
        CHECK(ChassisReleaseStartupDevices() == (uint8_t)!rollback_failed);
        CHECK(rollback_count == (rollback_failed ? 0u : 4u)); // 检查四台已登记轮电机。
        CHECK(ChassisReleaseStartupDevices() == 1u); // 重复清理不重复释放实例。
        return 0;
    }
#if defined(CHASSIS_BRIDGE_INTEGRATION)
    return test_bridge_to_chassis();
#endif
#ifdef CHASSIS_BOARD
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    for (unsigned i=0u;i<4u;++i) {
        motors[i].feedback_online=1u;
        motors[i].feedback_speed_valid=1u;
        motors[i].feedback_current_valid=1u;
    }
    // 未收到映射后的远端命令时保持停止；撤销已发布数据后立即停止。
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    MessagePublisher_t *bridge=MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    CHECK(bridge != NULL);
    Chassis_Ctrl_Cmd_s incoming={.vx=100.0f,.chassis_mode=CHASSIS_NO_FOLLOW};
    test_tick=10u;
    CHECK(MessagePublishAt(bridge,&incoming,10u,0u,0u,MESSAGE_CLOCK_LOCAL_MAPPED)==MESSAGE_OK);
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    test_tick=11u;
    CHECK(MessagePublishAt(bridge,&incoming,11u,0u,0u,MESSAGE_CLOCK_LOCAL_MAPPED)==MESSAGE_OK);
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(motors[i].enabled);
    test_tick=60u;
    tick_read_count=0u;
    advance_tick_on_read=3u;
    advance_tick_by_ms=2u;
    ChassisTask();
    advance_tick_on_read=0u;
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    CHECK(!RobotSafetyIsReady());
    CHECK(MessagePublisherInvalidate(bridge)==MESSAGE_OK);
    ChassisTask();
    for (unsigned i=0u;i<4u;++i) CHECK(!motors[i].enabled && motors[i].ref==0.0f);
    return 0;
#endif
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motor_configs[i].output_axis.gear_ratio == TEST_MECANUM_GEAR_RATIO);
        CHECK(motor_configs[i].controller_param_init_config.speed_PID.Kp > 10.0f);
    }
    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    Chassis_Ctrl_Cmd_s command = {
        .vx = 100.0f, .vy = 0.0f, .wz = 0.0f, .offset_angle = 0.0f,
        .chassis_mode = CHASSIS_NO_FOLLOW,
    };
    MessagePublishNow(commands, &command);
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[1].enabled && motors[2].enabled && motors[3].enabled);
    unsigned last_reference_write = 0;
    unsigned first_enable = UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motors[i].ref != 0.0f);
        if (motors[i].set_ref_order > last_reference_write) last_reference_write = motors[i].set_ref_order;
        if (motors[i].enable_order < first_enable) first_enable = motors[i].enable_order;
    }
    CHECK(last_reference_write < first_enable);
    CHECK(motors[0].ref > 0.0f && motors[2].ref < 0.0f);
#if defined(ONE_BOARD)
    command.safety_generation = 42u; // 测试用代际只属于主机输入。
    MessagePublishNow(commands, &command);
    ChassisTask();
    CHECK(observed_motion_generation == 42u); // 不使用任务当前时刻替换命令的原始代际。
    CHECK(observed_motion_deadline_ms == test_tick + MESSAGE_DEFAULT_MAX_AGE_MS); // 无额外期限的旧消息使用话题期限。
    reject_motion_reference = 1u; // 模拟命令被读取后许可发生变化。
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    reject_motion_reference = 0u;

    test_tick = UINT32_MAX - 2u; // 将短来源期限设在计数回绕后的 0 ms。
    CHECK(MessagePublishAtBounded(commands, &command, test_tick, test_tick, test_tick,
                                      MESSAGE_CLOCK_LOCAL, 3u) == MESSAGE_OK);
    test_tick = 0u; // 轮速计算开始时恰好还在来源期限内。
    ChassisTask();
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
    CHECK(observed_motion_deadline_ms == 0u); // 跨回绕的来源期限比话题期限更早。
    tick_read_count = 0u; // 本轮第二次取时发生在四轮目标计算之后。
    advance_tick_on_read = 2u;
    advance_tick_by_ms = 1u;
    ChassisTask();
    CHECK(test_tick == 1u); // 话题仍不过期，但短来源期限已到。
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);

    test_tick = 61u;
    CHECK(MessagePublishAtBounded(commands, &command, 30u, 30u, 60u, MESSAGE_CLOCK_LOCAL, 150u) == MESSAGE_OK);
    ChassisTask();
    CHECK(observed_motion_deadline_ms == 60u + MESSAGE_DEFAULT_MAX_AGE_MS); // 话题先到期时，使用原发布时间。
#endif
    command = (Chassis_Ctrl_Cmd_s){.offset_angle = 0.5f,
                                   .chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) CHECK(motors[i].ref < 0.0f);
    command.offset_angle = -0.5f;
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) CHECK(motors[i].ref > 0.0f);
#else
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    /* A zero-force command must clear old references as well as stop output. */
    command.chassis_mode = CHASSIS_ZERO_FORCE;
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f && motors[i].stop_count > 0);

#if CHASSIS_TASK_EXPECT_CONFIGURED
    /* Re-entry computes new bounded references before enabling the motors. */
    command = (Chassis_Ctrl_Cmd_s){.vx = 100000.0f, .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motors[i].enabled);
        CHECK(motors[i].enable_count > 0);
        CHECK(motors[i].ref != 0.0f);
        const float max_output_rad_s = CHASSIS_MAX_MOTOR_SPEED_DEG_S * DEGREE_2_RAD /
                                       TEST_MECANUM_GEAR_RATIO;
        CHECK(motors[i].ref <= max_output_rad_s && motors[i].ref >= -max_output_rad_s);
    }
    command = (Chassis_Ctrl_Cmd_s){.vx = 20.0f, .chassis_mode = (chassis_mode_e)99};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    command = (Chassis_Ctrl_Cmd_s){.vx = 20.0f, .offset_angle = 181.0f,
                                   .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    /* The 50 ms edge remains valid and the following millisecond revokes output. */
    test_tick = 100u;
    command = (Chassis_Ctrl_Cmd_s){.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    test_tick = 150u;
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
#endif
    test_tick = 151u;
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    test_tick = 152u;
    MessagePublishNow(commands, &command);
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
#else
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    puts("chassis task checks passed");
    return 0;
}
