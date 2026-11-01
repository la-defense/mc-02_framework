/** @file robot_can_bridge_test.c
 * @brief 测试双板实际启动入口、对时帧接收和供诊断读取的状态。
 */
#include "robot_can_bridge.h"
#include "robot_topics.h"
#include "can_time_sync.h"
#include "can_topic_wire.h"
#include "chassis_topic_codec.h"
#include "board_can_stm32.h"
#include "fdcan.h"
#include "test_check.h"
#if defined(CHASSIS_BOARD)
#include "robot_safety.h"
#endif

#include <string.h>

FDCAN_HandleTypeDef hcan1;
FDCAN_HandleTypeDef hcan2;
FDCAN_HandleTypeDef hcan3;
static uint32_t now_ms;
static CANInstance registered_can;
static void (*receive_callback)(CANInstance *);
static uint8_t frames[2][8];
static size_t frame_count;
static uint32_t next_id;
static FDCAN_HandleTypeDef *expected_handle;
static FDCAN_HandleTypeDef *serviced_handle;
static uint16_t expected_tx, expected_rx;

uint32_t HAL_GetTick(void) { return now_ms; }

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || receive_callback != NULL)
        return NULL;
    TEST_CHECK(config->can_handle == expected_handle && config->tx_id == expected_tx && config->rx_id == expected_rx);
    registered_can.can_handle = config->can_handle;
    registered_can.tx_id = config->tx_id;
    registered_can.id = config->id;
    receive_callback = config->can_module_callback;
    return &registered_can;
}

void CANUnregisterInstance(CANInstance *instance)
{
    if (instance == &registered_can)
        receive_callback = NULL;
}

void CANProcessTxBus(FDCAN_HandleTypeDef *handle) { serviced_handle=handle; }

CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *handle, const void *source,
    const CAN_TxFrame_t *input, size_t count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    TEST_CHECK(handle != NULL && source != NULL && input != NULL && request_id != NULL);
    TEST_CHECK(count > 0u && count <= 2u && priority == CAN_TX_PRIORITY_NORMAL && deadline_ms > 0u);
    frame_count = count;
    for (size_t i = 0u; i < count; ++i)
    {
        TEST_CHECK(input[i].length == 8u);
        memcpy(frames[i], input[i].data, 8u);
    }
    *request_id = ++next_id;
    return CAN_TX_SUBMIT_ACCEPTED;
}

CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *handle, const void *source,
    const CAN_TxFrame_t *input, size_t count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id)
{
    return CANSubmitInterruptibleBatch(handle,source,input,count,priority,expiry_ms-now_ms,request_id);
}

uint8_t CANGetTxRequestState(uint32_t request_id, CAN_TxResult_t *result)
{
    if (request_id == 0u || result == NULL)
        return 0u;
    memset(result, 0, sizeof(*result));
    result->state = CAN_TX_RESULT_COMPLETE;
    return 1u;
}

uint8_t CANCancelTxRequest(uint32_t request_id) { return request_id != 0u; }

#if defined(CHASSIS_BOARD)
static void receive_topic_message(const CanTopicMessage_t *message, uint8_t sequence)
{
    CanTopicFrame_t topic_frames[CAN_TOPIC_MAX_FRAMES];
    size_t topic_frame_count = 0u;
    TEST_CHECK(CanTopicEncode(message, sequence, topic_frames, CAN_TOPIC_MAX_FRAMES,
                              &topic_frame_count) == CAN_TOPIC_OK);
    for (size_t i = 0u; i < topic_frame_count; ++i)
    {
        memcpy(registered_can.rx_buff, topic_frames[i].bytes, topic_frames[i].length);
        registered_can.rx_len = topic_frames[i].length;
        registered_can.rx_timestamp_ms = now_ms;
        receive_callback(&registered_can);
    }
}
#endif

int main(void)
{
    TimeMappingState_t state = {.accepted_samples = 123u};
    uint32_t updated_ms = 456u;
    TEST_CHECK(!RobotCanBridgeGetTimeSyncState(&state, &updated_ms));
    TEST_CHECK(state.accepted_samples == 123u && updated_ms == 456u);
    RobotCanBridgeConfig_t config = {
        .can_bus=PLATFORM_CAN_BUS_2, .tx_id=0x421u, .rx_id=0x422u,
        .command_topic=ROBOT_TOPIC_CHASSIS_COMMAND, .feedback_topic=ROBOT_TOPIC_CHASSIS_FEEDBACK};
    expected_handle=&hcan2; expected_tx=config.tx_id; expected_rx=config.rx_id;
    TEST_CHECK(!RobotCanBridgeInit(NULL));
    TEST_CHECK(BoardCanResolve(PLATFORM_CAN_BUS_1) == &hcan1); // 第一条逻辑总线绑定本板 CAN1。
    TEST_CHECK(BoardCanResolve(PLATFORM_CAN_BUS_2) == &hcan2); // 第二条逻辑总线绑定本板 CAN2。
    TEST_CHECK(BoardCanResolve(PLATFORM_CAN_BUS_3) == &hcan3); // 第三条逻辑总线绑定本板 CAN3。
    RobotCanBridgeConfig_t missing = config; // 保留其余合法配置，只改变资源编号。
    missing.can_bus = PLATFORM_CAN_INVALID; // 未配置总线不能启动桥接。
    TEST_CHECK(!RobotCanBridgeInit(&missing) && receive_callback==NULL); // 失败前不注册设备。
    missing.can_bus = (PlatformCanBus_t)99; // 未声明的编号也不能落到默认总线。
    TEST_CHECK(!RobotCanBridgeInit(&missing) && receive_callback==NULL); // 无隐式回退。
    RobotCanBridgeConfig_t invalid=config;
    invalid.command_topic=0u;
    TEST_CHECK(!RobotCanBridgeInit(&invalid) && receive_callback==NULL);
    invalid=config; invalid.rx_id=invalid.tx_id;
    TEST_CHECK(!RobotCanBridgeInit(&invalid) && receive_callback==NULL);
    invalid=config; invalid.feedback_topic=0u;
    TEST_CHECK(!RobotCanBridgeInit(&invalid) && receive_callback==NULL);
    invalid=config; invalid.rx_id=0x800u;
    TEST_CHECK(!RobotCanBridgeInit(&invalid) && receive_callback==NULL);
    TEST_CHECK(RobotCanBridgeInit(&config));
    RobotCanBridgeServiceTx();
    TEST_CHECK(serviced_handle==expected_handle);
    TEST_CHECK(!RobotCanBridgeGetTimeSyncState(&state, &updated_ms));

    CANInstance peer_can = {.can_handle = &hcan1, .tx_id = 0x311u};
    CanTimeSync_t peer = {0};
    TEST_CHECK(CanTimeSyncInit(&peer, &peer_can));
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        const uint32_t local = 1000u + i * 200u;
        now_ms = local;
        RobotCanBridgeService();
        TEST_CHECK(frame_count == 1u);
        uint8_t probe[8];
        memcpy(probe, frames[0], 8u);
        TEST_CHECK(CanTimeSyncReceive(&peer, probe, 8u, local + 1002u,
                                      local + 1002u) == CAN_TIME_SYNC_OK);
        TEST_CHECK(frame_count == 2u);
        uint8_t replies[2][8];
        memcpy(replies, frames, sizeof(replies));
        for (size_t j = 0u; j < 2u; ++j)
        {
            memcpy(registered_can.rx_buff, replies[j], 8u);
            registered_can.rx_len = 8u;
            registered_can.rx_timestamp_ms = local + 4u;
            now_ms = local + 4u;
            receive_callback(&registered_can);
        }
    }
    RobotCanBridgeService();
    TEST_CHECK(RobotCanBridgeGetTimeSyncState(&state, &updated_ms));
    TEST_CHECK(state.synchronized && state.accepted_samples == 3u && updated_ms == 1404u);

    // 诊断读取不能把超过5ms未更新的状态继续显示为同步。
    now_ms = 1410u;
    TEST_CHECK(RobotCanBridgeGetTimeSyncState(&state, &updated_ms) && !state.synchronized);
    RobotCanBridgeService();
    TEST_CHECK(RobotCanBridgeGetTimeSyncState(&state, &updated_ms) && state.synchronized);
#if defined(CHASSIS_BOARD)
    // 新停车命令到达时，底盘任务尚未运行，发送侧也必须先看到许可撤销。
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetySnapshot_t safety_snapshot = {0};
    TEST_CHECK(RobotSafetyGetSnapshot(&safety_snapshot));
    now_ms = 1410u;
    TEST_CHECK(RobotSafetyTryGrantRemoteChassisPermit(
        safety_snapshot.chassis_inhibit_generation, now_ms, 50u, now_ms));
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReadyAt(now_ms));

    Chassis_Ctrl_Cmd_s chassis_command = {.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW};
    CanTopicMessage_t chassis_message = {
        .topic_id = ROBOT_TOPIC_CHASSIS_COMMAND,
        .length = CHASSIS_COMMAND_WIRE_BYTES,
        .source_ms = 2409u,
        .first_rx_ms = now_ms,
    };
    TEST_CHECK(ChassisCommandEncode(&chassis_command, chassis_message.payload,
                                   sizeof(chassis_message.payload)) == CHASSIS_CODEC_OK);
    receive_topic_message(&chassis_message, 40u);
    TEST_CHECK(RobotSafetyIsReadyAt(now_ms));

    chassis_command = (Chassis_Ctrl_Cmd_s){.chassis_mode = CHASSIS_ZERO_FORCE};
    chassis_message.source_ms = 2410u;
    TEST_CHECK(ChassisCommandEncode(&chassis_command, chassis_message.payload,
                                   sizeof(chassis_message.payload)) == CHASSIS_CODEC_OK);
    receive_topic_message(&chassis_message, 41u);
    TEST_CHECK(!RobotSafetyIsReadyAt(now_ms));
#endif
    now_ms = 1905u;
    TEST_CHECK(RobotCanBridgeGetTimeSyncState(&state, &updated_ms) && !state.synchronized);
#if defined(CHASSIS_BOARD)
    // 对时失效会通过桥接撤销回调即时清除许可，不等待底盘任务轮询话题。
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    safety_snapshot = (RobotSafetySnapshot_t){0};
    TEST_CHECK(RobotSafetyGetSnapshot(&safety_snapshot));
    TEST_CHECK(RobotSafetyTryGrantRemoteChassisPermit(
        safety_snapshot.chassis_inhibit_generation, now_ms, 50u, now_ms));
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReadyAt(now_ms));
    RobotCanBridgeService();
    TEST_CHECK(!RobotSafetyIsReadyAt(now_ms));
#endif
    return 0;
}
