/** @file can_time_sync.c
 * @brief 通过 CAN 队列交换四个时刻，并复用 TimeMapping 计算时钟偏差。
 */
#include "can_time_sync.h"
#include "crc.h"

#include <limits.h>
#include <stddef.h>

#define CAN_TIME_PROBE 0xc1u
#define CAN_TIME_REPLY_RECEIVED 0xc2u
#define CAN_TIME_REPLY_SENT 0xc3u
#define CAN_TIME_PERIOD_MS 200u
#define CAN_TIME_EXCHANGE_MS 80u
#define CAN_TIME_MAX_RTT_MS 20u
#define CAN_TIME_MAX_JUMP_MS 20u
#define CAN_TIME_RELATIVE_PPM 2000u

static uint32_t ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

static uint8_t MakeFrame(uint8_t *data, uint8_t type, uint16_t sequence, uint32_t time_ms)
{
    data[0] = type;
    data[1] = (uint8_t)sequence;
    data[2] = (uint8_t)(sequence >> 8u);
    for (uint8_t i = 0u; i < 4u; ++i)
        data[3u + i] = (uint8_t)(time_ms >> (8u * i));
    uint32_t crc = 0u;
    if (CRCCompute(CRC8_CAN_A5, data, 7u, &crc) != CRC_OK)
        return 0u;
    data[7] = (uint8_t)crc;
    return 1u;
}

static uint8_t RequestFinished(uint32_t *request_id)
{
    if (*request_id == 0u)
        return 1u;
    CAN_TxResult_t result;
    if (!CANGetTxRequestState(*request_id, &result) ||
        (result.pending_frames == 0u &&
         (result.state == CAN_TX_RESULT_COMPLETE || result.state == CAN_TX_RESULT_FAILED ||
          result.state == CAN_TX_RESULT_EXPIRED || result.state == CAN_TX_RESULT_CANCELLED ||
          result.state == CAN_TX_RESULT_SUPERSEDED)))
    {
        *request_id = 0u;
        return 1u;
    }
    return 0u;
}

static uint8_t Submit(CanTimeSync_t *sync, uint8_t data[][8], size_t count, uint32_t *request_id)
{
    CAN_TxFrame_t frames[2];
    for (size_t i = 0u; i < count; ++i)
    {
        frames[i].identifier = sync->can->tx_id;
        frames[i].identifier_type = CAN_TX_ID_STANDARD;
        frames[i].data = data[i];
        frames[i].length = 8u;
    }
    return (uint8_t)(CANSubmitInterruptibleBatch(sync->can->can_handle, sync, frames, count,
                      CAN_TX_PRIORITY_NORMAL, CAN_TIME_EXCHANGE_MS, request_id) ==
                      CAN_TX_SUBMIT_ACCEPTED);
}

uint8_t CanTimeSyncInit(CanTimeSync_t *sync, CANInstance *can)
{
    if (sync == NULL || can == NULL || can->can_handle == NULL || can->tx_id > 0x7ffu)
        return 0u;
    CanTimeSync_t candidate = {0};
    const TimeMappingConfig_t config = {CAN_TIME_MAX_RTT_MS, CAN_TIME_EXCHANGE_MS,
                                        CAN_TIME_SYNC_VALID_MS, CAN_TIME_MAX_JUMP_MS};
    if (TimeMappingInit(&candidate.mapping, &config) != TIME_MAPPING_OK)
        return 0u;
    candidate.can = can;
    candidate.initialized = 1u;
    *sync = candidate;
    return 1u;
}

CanTimeSyncStatus_t CanTimeSyncService(CanTimeSync_t *sync, uint32_t now_ms)
{
    if (sync == NULL || !sync->initialized || sync->can == NULL)
        return CAN_TIME_SYNC_ARGUMENT;
    if (sync->waiting)
    {
        if ((uint32_t)(now_ms - sync->probe_t1_ms) <= CAN_TIME_EXCHANGE_MS)
            return CAN_TIME_SYNC_WAITING;
        // 超期后取消未发送部分；硬件中仍有帧时不能开始下一次交换。
        (void)CANCancelTxRequest(sync->probe_request_id);
        sync->waiting = 0u;
        sync->has_t2 = 0u;
    }
    if (!RequestFinished(&sync->probe_request_id))
        return CAN_TIME_SYNC_BUSY;
    if (sync->has_sent && (uint32_t)(now_ms - sync->previous_probe_ms) < CAN_TIME_PERIOD_MS)
        return CAN_TIME_SYNC_WAITING;

    uint8_t frames[1][8];
    const uint16_t sequence = sync->next_sequence;
    // 队列满时也按周期重试，避免在每次电机任务中持续提交。
    sync->previous_probe_ms = now_ms;
    sync->has_sent = 1u;
    if (!MakeFrame(frames[0], CAN_TIME_PROBE, sequence, now_ms) ||
        !Submit(sync, frames, 1u, &sync->probe_request_id))
        return CAN_TIME_SYNC_SEND_ERROR;
    sync->probe_t1_ms = now_ms;
    sync->pending_sequence = sequence;
    sync->next_sequence = (uint16_t)(sequence + 1u);
    sync->waiting = 1u;
    sync->has_t2 = 0u;
    return CAN_TIME_SYNC_OK;
}

CanTimeSyncStatus_t CanTimeSyncReceive(CanTimeSync_t *sync, const uint8_t *data,
                                       uint8_t length, uint32_t received_ms, uint32_t now_ms)
{
    if (sync == NULL || !sync->initialized || sync->can == NULL || data == NULL)
        return CAN_TIME_SYNC_ARGUMENT;
    uint32_t crc = 0u;
    if (length != 8u || (uint32_t)(now_ms - received_ms) > CAN_TIME_EXCHANGE_MS ||
        CRCCompute(CRC8_CAN_A5, data, 7u, &crc) != CRC_OK || (uint8_t)crc != data[7])
        return CAN_TIME_SYNC_FRAME_ERROR;
    const uint16_t sequence = (uint16_t)(data[1] | ((uint16_t)data[2] << 8u));
    const uint32_t timestamp = ReadU32(data + 3u);
    if (data[0] == CAN_TIME_PROBE)
    {
        const uint32_t advance = timestamp - sync->last_peer_ms;
        const uint8_t peer_reset = (uint8_t)(sync->has_peer_time && advance >= 0x80000000u);
        if (peer_reset)
        {
            // 对端主动探测暴露了回退的时钟；立即废弃旧映射和本次未完成请求。
            (void)TimeMappingReset(&sync->mapping);
            (void)CANCancelTxRequest(sync->probe_request_id);
            sync->waiting = 0u;
            sync->has_t2 = 0u;
            sync->has_peer_time = 0u;
        }
        // 同时只保留一组响应，队列中旧响应尚未结束时丢弃新请求。
        if (!RequestFinished(&sync->response_request_id))
            return peer_reset ? CAN_TIME_SYNC_PEER_RESET : CAN_TIME_SYNC_BUSY;
        uint8_t frames[2][8];
        if (!MakeFrame(frames[0], CAN_TIME_REPLY_RECEIVED, sequence, received_ms) ||
            !MakeFrame(frames[1], CAN_TIME_REPLY_SENT, sequence, now_ms) ||
            !Submit(sync, frames, 2u, &sync->response_request_id))
            return peer_reset ? CAN_TIME_SYNC_PEER_RESET : CAN_TIME_SYNC_SEND_ERROR;
        return peer_reset ? CAN_TIME_SYNC_PEER_RESET : CAN_TIME_SYNC_OK;
    }
    if (!sync->waiting || sequence != sync->pending_sequence ||
        (uint32_t)(received_ms - sync->probe_t1_ms) > CAN_TIME_EXCHANGE_MS ||
        (uint32_t)(now_ms - sync->probe_t1_ms) > CAN_TIME_EXCHANGE_MS)
        return CAN_TIME_SYNC_FRAME_ERROR;
    if (data[0] == CAN_TIME_REPLY_RECEIVED && !sync->has_t2)
    {
        sync->peer_t2_ms = timestamp;
        sync->reply_t2_received_ms = received_ms;
        sync->has_t2 = 1u;
        return CAN_TIME_SYNC_WAITING;
    }
    if (data[0] != CAN_TIME_REPLY_SENT || !sync->has_t2 ||
        (uint32_t)(received_ms - sync->reply_t2_received_ms) > CAN_TIME_EXCHANGE_MS)
    {
        // 当前请求的响应重复或错序后整次作废，不能再用后续帧补齐。
        sync->waiting = 0u;
        sync->has_t2 = 0u;
        return CAN_TIME_SYNC_FRAME_ERROR;
    }
    sync->waiting = 0u;
    sync->has_t2 = 0u;

    const uint32_t elapsed = timestamp - sync->last_peer_ms;
    const uint8_t peer_reset = (uint8_t)(sync->has_peer_time &&
                                          (elapsed == 0u || elapsed >= 0x80000000u));
    if (peer_reset)
        (void)TimeMappingReset(&sync->mapping);
    if (TimeMappingObserveExchange(&sync->mapping, sync->probe_t1_ms, sync->peer_t2_ms,
                                   timestamp, received_ms) != TIME_MAPPING_OK)
        return peer_reset ? CAN_TIME_SYNC_PEER_RESET : CAN_TIME_SYNC_FRAME_ERROR;
    sync->last_peer_ms = timestamp;
    sync->has_peer_time = 1u;
    return peer_reset ? CAN_TIME_SYNC_PEER_RESET : CAN_TIME_SYNC_OK;
}

TimeMappingStatus_t CanTimeSyncGetState(const CanTimeSync_t *sync, uint32_t now_ms,
                                       TimeMappingState_t *output)
{
    if (sync == NULL || output == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    if (!sync->initialized)
        return TIME_MAPPING_ERR_UNINITIALIZED;
    return TimeMappingGetState(&sync->mapping, now_ms, output);
}

uint8_t CanTimeSyncMap(void *context, uint32_t remote_ms, uint32_t now_ms,
                       uint32_t *local_ms, uint32_t *uncertainty_ms)
{
    CanTimeSync_t *sync = context;
    if (sync == NULL || !sync->initialized || local_ms == NULL || uncertainty_ms == NULL)
        return 0u;
    uint32_t converted = 0u;
    if (TimeMappingRemoteToLocal(&sync->mapping, remote_ms, now_ms, &converted) != TIME_MAPPING_OK)
        return 0u;
    const uint32_t raw_distance = remote_ms - sync->last_peer_ms;
    if (raw_distance == 0x80000000u)
        return 0u;
    const uint64_t distance = raw_distance <= INT32_MAX ? raw_distance :
                              0x100000000ULL - raw_distance;
    // 三份样本最早可能比最新样本早两段有效期；再加时钟相对频率误差。
    const uint64_t drift = ((distance + 2u * CAN_TIME_SYNC_VALID_MS + CAN_TIME_EXCHANGE_MS) *
                            CAN_TIME_RELATIVE_PPM + 999999u) / 1000000u;
    const uint64_t uncertainty = (CAN_TIME_MAX_RTT_MS + 1u) / 2u + 2u + drift;
    if (uncertainty > INT32_MAX)
        return 0u;
    *local_ms = converted;
    *uncertainty_ms = (uint32_t)uncertainty;
    return 1u;
}
