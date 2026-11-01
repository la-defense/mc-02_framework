#include "chassis.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "robot_def.h"
#include "referee_task.h"
#include "super_cap.h"
#include "general_def.h"
#ifdef CHASSIS_BOARD
#include "robot_safety.h"
#endif
#if defined(CHASSIS_BRIDGE_INTEGRATION)
#include "can_topic_link.h"
#include "chassis_topic_codec.h"
#endif

#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

static uint32_t test_tick;
static uint32_t tick_read_count;
static uint32_t advance_tick_on_read;
static uint32_t advance_tick_by_ms;
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
static DJIMotorInstance motors[4];
static Motor_Init_Config_s motor_configs[4];
static unsigned motor_count;
static unsigned operation_order;
static SuperCapInstance cap;
static int fail_cap, fail_motor;
#ifdef CHASSIS_BOARD
static attitude_t chassis_attitude;
attitude_t *INS_GetStorage(void) { return &chassis_attitude; }
#endif

DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    if (fail_motor) return NULL;
    if (motor_count >= 4)
        return NULL;
    motor_configs[motor_count] = *config;
    return &motors[motor_count++];
}
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{ if (motor) { motor->ref = ref; motor->set_ref_order = ++operation_order; } }
void DJIMotorStop(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 0; ++motor->stop_count; } }
void DJIMotorEnable(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 1; ++motor->enable_count; motor->enable_order = ++operation_order; } }
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{ if (summary) { memset(summary, 0, sizeof(*summary)); if (motor) summary->online = motor->feedback_online; } }
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(DJIMotorInstance *motor, uint32_t now_ms,
                                                     Motor_Feedback_Snapshot_s *snapshot)
{
    (void)now_ms;
    if (motor == NULL || snapshot == NULL) return MOTOR_FEEDBACK_BAD_ARGUMENT;
    snapshot->speed.valid = motor->feedback_speed_valid;
    snapshot->current.valid = motor->feedback_current_valid;
    if (motor->feedback_online == 0u) return MOTOR_FEEDBACK_NOT_READY;
    if (motor->feedback_age_ms > 20u) return MOTOR_FEEDBACK_STALE;
    return MOTOR_FEEDBACK_OK;
}
SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *config) { (void)config; return fail_cap ? NULL : &cap; }
RefereeStatus_t RefereeInit(UART_HandleTypeDef *handle, const RefereeConfig_t *config)
{ (void)handle; (void)config; return REFEREE_OK; }
void MC02ChassisTestLogError(void) {}

#if defined(CHASSIS_BRIDGE_INTEGRATION)
static FDCAN_HandleTypeDef bridge_can_handle;
static CANInstance bridge_can;
static void (*bridge_receive)(CANInstance *);
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
    const CAN_Init_Config_s config={.can_handle=&bridge_can_handle,.tx_id=0x311u,.rx_id=0x312u};
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

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "cap-failure") == 0) fail_cap = 1;
    if (argc == 2 && strcmp(argv[1], "motor-failure") == 0) fail_motor = 1;
    uint8_t initialized = ChassisInit();
    // 未实现普通功率回退前，电容注册或核心电机失败都保留原失败结果。
    if (fail_cap || fail_motor) { CHECK(initialized == 0u); return 0; }
    CHECK(initialized != 0u);
    CHECK(motor_count == 4);
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
        CHECK(motor_configs[i].output_axis.gear_ratio == REDUCTION_RATIO_WHEEL);
        CHECK(motor_configs[i].output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE);
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
                                       REDUCTION_RATIO_WHEEL;
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
