/** @file can_time_sync_test.c
 * @brief 用两套生产对时实例和模拟 CAN 线路检查不同起点下的时间换算。
 */
#include "can_time_sync.h"
#include "bsp_can.h" // 此测试用旧 CAN 对象提供两个不同的端点标识。
#include "test_check.h"

#include <string.h>

static uint8_t sent[2][8];
static size_t sent_count;
static uint32_t next_request_id;
static uint32_t send_attempts;
static CAN_TxSubmitStatus_t next_send_result = CAN_TX_SUBMIT_ACCEPTED;

// 模拟 CAN 队列在提交时复制完整帧；测试随后决定另一块板何时收到。
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *handle, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    TEST_CHECK(handle != NULL && source != NULL && frames != NULL && request_id != NULL);
    TEST_CHECK(count > 0u && count <= 2u && priority == CAN_TX_PRIORITY_NORMAL && deadline_ms > 0u);
    ++send_attempts;
    if (next_send_result != CAN_TX_SUBMIT_ACCEPTED)
        return next_send_result;
    sent_count = count;
    for (size_t i = 0u; i < count; ++i)
    {
        TEST_CHECK(frames[i].length == 8u);
        memcpy(sent[i], frames[i].data, sizeof(sent[i]));
    }
    *request_id = ++next_request_id;
    return CAN_TX_SUBMIT_ACCEPTED;
}

uint8_t CANGetTxRequestState(uint32_t request_id, CAN_TxResult_t *result)
{
    if (request_id == 0u || result == NULL)
        return 0u;
    memset(result, 0, sizeof(*result));
    result->state = CAN_TX_RESULT_COMPLETE;
    return 1u;
}

CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *handle, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id)
{
    // 对时测试不调用绝对期限提交；保留接口供共享平台替身链接。
    return CANSubmitInterruptibleBatch(handle, source, frames, count, priority, expiry_ms, request_id);
}

uint8_t CANCancelTxRequest(uint32_t request_id)
{
    return request_id != 0u;
}

int main(void)
{
    FDCAN_HandleTypeDef handle_a = {0};
    FDCAN_HandleTypeDef handle_b = {0};
    CANInstance can_a = {.can_handle = &handle_a, .tx_id = 0x312u};
    CANInstance can_b = {.can_handle = &handle_b, .tx_id = 0x311u};
    CanTimeSync_t board_a = {0};
    CanTimeSync_t board_b = {0};
    // 失败后对象保持未初始化，随后可以用有效配置重试。
    TEST_CHECK(!CanTimeSyncInit(&board_a, PLATFORM_CAN_INVALID, can_a.tx_id, &can_a));
    TEST_CHECK(!CanTimeSyncInit(&board_a, PLATFORM_CAN_BUS_1, 0x800u, &can_a));
    TEST_CHECK(!board_a.initialized);
    TEST_CHECK(CanTimeSyncInit(&board_a, PLATFORM_CAN_BUS_1, can_a.tx_id, &can_a));
    TEST_CHECK(CanTimeSyncInit(&board_b, PLATFORM_CAN_BUS_2, can_b.tx_id, &can_b));

    uint32_t mapped = 77u;
    uint32_t uncertainty = 88u;
    TEST_CHECK(!CanTimeSyncMap(&board_a, 2002u, 1004u, &mapped, &uncertainty));
    TEST_CHECK(mapped == 77u && uncertainty == 88u);

    // B 的时钟始终比 A 快 1000 ms；单向传输各用 2 ms。
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        const uint32_t a_send = 1000u + 200u * i;
        TEST_CHECK(CanTimeSyncService(&board_a, a_send) == CAN_TIME_SYNC_OK);
        TEST_CHECK(sent_count == 1u);
        if (i == 0u)
        {
            const uint8_t expected_probe[8] = {0xc1u,0u,0u,0xe8u,0x03u,0u,0u,0xbcu};
            TEST_CHECK(memcmp(sent[0], expected_probe, 8u) == 0);
        }
        uint8_t probe[8];
        memcpy(probe, sent[0], sizeof(probe));
        TEST_CHECK(CanTimeSyncReceive(&board_b, probe, sizeof(probe),
                                      a_send + 1002u, a_send + 1002u) == CAN_TIME_SYNC_OK);
        TEST_CHECK(sent_count == 2u);
        uint8_t response[2][8];
        memcpy(response, sent, sizeof(response));
        if (i == 0u)
        {
            const uint8_t expected_first[8] = {0xc2u,0u,0u,0xd2u,0x07u,0u,0u,0xccu};
            const uint8_t expected_second[8] = {0xc3u,0u,0u,0xd2u,0x07u,0u,0u,0x25u};
            const uint8_t wrong_sequence[8] = {0xc2u,1u,0u,0xd2u,0x07u,0u,0u,0x9bu};
            TEST_CHECK(memcmp(response[0], expected_first, 8u) == 0);
            TEST_CHECK(memcmp(response[1], expected_second, 8u) == 0);
            TEST_CHECK(CanTimeSyncReceive(&board_a, wrong_sequence, 8u,
                                          a_send + 4u, a_send + 4u) == CAN_TIME_SYNC_FRAME_ERROR);
            uint8_t damaged[8]; memcpy(damaged, response[0], 8u); damaged[7] ^= 1u;
            TEST_CHECK(CanTimeSyncReceive(&board_a, damaged, 8u,
                                          a_send + 4u, a_send + 4u) == CAN_TIME_SYNC_FRAME_ERROR);
        }
        TEST_CHECK(CanTimeSyncReceive(&board_a, response[0], 8u,
                                      a_send + 4u, a_send + 4u) == CAN_TIME_SYNC_WAITING);
        TEST_CHECK(CanTimeSyncReceive(&board_a, response[1], 8u,
                                      a_send + 4u, a_send + 4u) == CAN_TIME_SYNC_OK);
    }
    TEST_CHECK(CanTimeSyncMap(&board_a, 2402u, 1404u, &mapped, &uncertainty));
    TEST_CHECK(mapped == 1402u && uncertainty >= 2u && uncertainty < 50u);
    TimeMappingState_t state = {0};
    TEST_CHECK(CanTimeSyncGetState(&board_a, 1404u, &state) == TIME_MAPPING_OK);
    TEST_CHECK(state.synchronized && state.accepted_samples == 3u);
    TEST_CHECK(CanTimeSyncGetState(&board_a, 1905u, &state) == TIME_MAPPING_OK);
    TEST_CHECK(!state.synchronized);
    mapped = 77u; uncertainty = 88u;
    TEST_CHECK(!CanTimeSyncMap(&board_a, 2402u, 1905u, &mapped, &uncertainty));
    TEST_CHECK(mapped == 77u && uncertainty == 88u);

    // 队列满时不应每次任务调用都重试，恢复后仍可再次发起探测。
    CanTimeSync_t blocked = {0};
    TEST_CHECK(CanTimeSyncInit(&blocked, PLATFORM_CAN_BUS_1, can_a.tx_id, &can_a));
    next_send_result = CAN_TX_SUBMIT_FULL;
    const uint32_t before = send_attempts;
    TEST_CHECK(CanTimeSyncService(&blocked, 5000u) == CAN_TIME_SYNC_SEND_ERROR);
    TEST_CHECK(CanTimeSyncService(&blocked, 5001u) == CAN_TIME_SYNC_WAITING);
    TEST_CHECK(send_attempts == before + 1u);
    next_send_result = CAN_TX_SUBMIT_ACCEPTED;
    TEST_CHECK(CanTimeSyncService(&blocked, 5200u) == CAN_TIME_SYNC_OK);

    // 同一请求的第一帧重复后，随后的第二帧不能组成有效样本。
    CanTimeSync_t receiver = {0};
    CanTimeSync_t sender = {0};
    TEST_CHECK(CanTimeSyncInit(&receiver, PLATFORM_CAN_BUS_1, can_a.tx_id, &can_a));
    TEST_CHECK(CanTimeSyncInit(&sender, PLATFORM_CAN_BUS_2, can_b.tx_id, &can_b));
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        const uint32_t local = 10000u + 200u * i;
        TEST_CHECK(CanTimeSyncService(&receiver, local) == CAN_TIME_SYNC_OK);
        uint8_t probe[8];
        memcpy(probe, sent[0], sizeof(probe));
        TEST_CHECK(CanTimeSyncReceive(&sender, probe, 8u, local + 1002u,
                                      local + 1002u) == CAN_TIME_SYNC_OK);
        uint8_t replies[2][8];
        memcpy(replies, sent, sizeof(replies));
        TEST_CHECK(CanTimeSyncReceive(&receiver, replies[0], 8u, local + 4u,
                                      local + 4u) == CAN_TIME_SYNC_WAITING);
        if (i == 0u)
        {
            TEST_CHECK(CanTimeSyncReceive(&receiver, replies[0], 8u, local + 4u,
                                          local + 4u) == CAN_TIME_SYNC_FRAME_ERROR);
            TEST_CHECK(CanTimeSyncReceive(&receiver, replies[1], 8u, local + 4u,
                                          local + 4u) == CAN_TIME_SYNC_FRAME_ERROR);
        }
        else
            TEST_CHECK(CanTimeSyncReceive(&receiver, replies[1], 8u, local + 4u,
                                          local + 4u) == CAN_TIME_SYNC_OK);
    }
    TEST_CHECK(!CanTimeSyncMap(&receiver, 11402u, 10404u, &mapped, &uncertainty));

    // 第二帧携带的本地入队时间不能早于第一帧，即使两帧内容和 CRC 都有效。
    CanTimeSync_t reordered = {0};
    TEST_CHECK(CanTimeSyncInit(&reordered, PLATFORM_CAN_BUS_1, can_a.tx_id, &can_a));
    TEST_CHECK(CanTimeSyncService(&reordered, 20000u) == CAN_TIME_SYNC_OK);
    uint8_t probe[8];
    memcpy(probe, sent[0], sizeof(probe));
    TEST_CHECK(CanTimeSyncReceive(&sender, probe, 8u, 21002u, 21002u) == CAN_TIME_SYNC_OK);
    uint8_t replies[2][8];
    memcpy(replies, sent, sizeof(replies));
    TEST_CHECK(CanTimeSyncReceive(&reordered, replies[0], 8u, 20010u, 20010u) == CAN_TIME_SYNC_WAITING);
    TEST_CHECK(CanTimeSyncReceive(&reordered, replies[1], 8u, 20009u, 20010u) == CAN_TIME_SYNC_FRAME_ERROR);

    // 两板的毫秒计数跨越32位回绕后，仍按原来的相对偏差换算。
    CanTimeSync_t wrap_a = {0}, wrap_b = {0};
    TEST_CHECK(CanTimeSyncInit(&wrap_a, PLATFORM_CAN_BUS_1, can_a.tx_id, &can_a) && CanTimeSyncInit(&wrap_b, PLATFORM_CAN_BUS_2, can_b.tx_id, &can_b));
    for (uint32_t i = 0u; i < 3u; ++i)
    {
        const uint32_t local = UINT32_MAX - 300u + i * 200u;
        TEST_CHECK(CanTimeSyncService(&wrap_a, local) == CAN_TIME_SYNC_OK);
        memcpy(probe, sent[0], sizeof(probe));
        TEST_CHECK(CanTimeSyncReceive(&wrap_b, probe, 8u, local + 1002u,
                                      local + 1002u) == CAN_TIME_SYNC_OK);
        memcpy(replies, sent, sizeof(replies));
        TEST_CHECK(CanTimeSyncReceive(&wrap_a, replies[0], 8u, local + 4u,
                                      local + 4u) == CAN_TIME_SYNC_WAITING);
        TEST_CHECK(CanTimeSyncReceive(&wrap_a, replies[1], 8u, local + 4u,
                                      local + 4u) == CAN_TIME_SYNC_OK);
    }
    const uint32_t last = UINT32_MAX - 300u + 2u * 200u;
    TEST_CHECK(CanTimeSyncMap(&wrap_a, last + 1002u, last + 4u, &mapped, &uncertainty));
    TEST_CHECK(mapped == last + 2u && uncertainty < 50u);
    return 0;
}
