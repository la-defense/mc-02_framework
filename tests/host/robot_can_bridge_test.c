/** @file robot_can_bridge_test.c
 * @brief 测试双板实际启动入口、对时帧接收和供诊断读取的状态。
 */
#include "robot_can_bridge.h"
#include "robot_topics.h"
#include "can_time_sync.h"
#include "fdcan.h"
#include "test_check.h"

#include <string.h>

FDCAN_HandleTypeDef hcan1;
FDCAN_HandleTypeDef hcan2;
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

int main(void)
{
    TimeMappingState_t state = {.accepted_samples = 123u};
    uint32_t updated_ms = 456u;
    TEST_CHECK(!RobotCanBridgeGetTimeSyncState(&state, &updated_ms));
    TEST_CHECK(state.accepted_samples == 123u && updated_ms == 456u);
    RobotCanBridgeConfig_t config = {
        .can_handle=&hcan2, .tx_id=0x421u, .rx_id=0x422u,
        .command_topic=ROBOT_TOPIC_CHASSIS_COMMAND, .feedback_topic=ROBOT_TOPIC_CHASSIS_FEEDBACK};
    expected_handle=config.can_handle; expected_tx=config.tx_id; expected_rx=config.rx_id;
    TEST_CHECK(!RobotCanBridgeInit(NULL));
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
    TEST_CHECK(serviced_handle==config.can_handle);
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
    now_ms = 1905u;
    TEST_CHECK(RobotCanBridgeGetTimeSyncState(&state, &updated_ms) && !state.synchronized);
    return 0;
}
