/** @file can_comm_freshness_test.c
 * @brief 用生产 CRC 验证 CAN 拼包和原接收时间，独立运行的主机测试。
 */
#include "crc.h"
#include "crc_injection.h"
#include "test_check.h"
#include "can_comm.h"
#include <stdlib.h>
#include <string.h>

static uint32_t now_ms;
static uint8_t daemon_online;
static uint32_t sends;
static CANInstance can_instance;
static DaemonInstance daemon_instance;
static void (*rx_callback)(CANInstance *);

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config) { (void)config; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; daemon_online = 1u; }
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{ (void)received_ms; if (instance != &daemon_instance) return DAEMON_ERR_NOT_REGISTERED; if ((flags & DAEMON_REPORT_TRANSPORT_VALID) != 0u) daemon_online = 1u; return DAEMON_OK; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return daemon_online; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    can_instance.can_handle = config->can_handle;
    can_instance.tx_id = config->tx_id;
    can_instance.id = config->id;
    rx_callback = config->can_module_callback;
    return &can_instance;
}
CAN_TxSubmitStatus_t CANSubmitOneShotBatch(FDCAN_HandleTypeDef *hcan, const void *source,
                                           const CAN_TxFrame_t *frames, size_t frame_count,
                                           CAN_TxPriority_t priority, uint32_t deadline_ms,
                                           uint32_t *request_id)
{
    (void)hcan; (void)source; (void)priority; (void)deadline_ms;
    if (frames == NULL || frame_count == 0u || request_id == NULL)
        return CAN_TX_SUBMIT_INVALID;
    sends += (uint32_t)frame_count;
    *request_id = 1u;
    return CAN_TX_SUBMIT_ACCEPTED;
}
static void deliver(uint32_t timestamp_ms, const uint8_t *bytes, uint8_t length)
{
    now_ms = timestamp_ms;
    memset(can_instance.rx_buff, 0, sizeof(can_instance.rx_buff));
    memcpy(can_instance.rx_buff, bytes, length);
    can_instance.rx_len = length;
    can_instance.rx_timestamp_ms = timestamp_ms;
    rx_callback(&can_instance);
}

int main(void)
{
    CANComm_Init_Config_s config = {.send_data_len = 4u, .recv_data_len = 4u, .daemon_timeout_ms = 50u};
    CANCommInstance *comm = CANCommInit(&config);
    TEST_CHECK(comm != NULL && rx_callback != NULL);
    uint8_t payload[4] = {1u, 2u, 3u, 4u};
    uint8_t frame[8] = {CAN_COMM_HEADER, 4u, 1u, 2u, 3u, 4u, 10u, CAN_COMM_TAIL};
    uint32_t crc;
    TEST_CHECK(CRCCompute(CRC8_CAN_A5, payload, sizeof(payload), &crc) == CRC_OK);
    frame[6] = (uint8_t)crc;
    uint8_t output[4];
    MessageFreshness_t freshness;

    TEST_CHECK(CANCommIsOnline(comm) == 0u);
    uint8_t snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 0u, 50u, &freshness);
    TEST_CHECK(snapshot_valid == 0u);
    deliver(0u, frame, sizeof(frame));
    snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 50u, 50u, &freshness);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(memcmp(output, payload, sizeof(payload)) == 0 && freshness.valid && freshness.sequence == 1u);
    TEST_CHECK(CANCommIsOnline(comm));
    now_ms = 51u;
    snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 51u, 50u, NULL);
    TEST_CHECK(snapshot_valid == 0u);
    TEST_CHECK(output[0] == 0u && !CANCommIsOnline(comm));

    /* Bad CRC must not refresh a previously expired snapshot. */
    frame[6]++;
    deliver(60u, frame, sizeof(frame));
    snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 60u, 50u, &freshness);
    TEST_CHECK(snapshot_valid == 0u);
    TEST_CHECK(!freshness.valid);
    frame[6] = (uint8_t)crc;
    deliver(61u, frame, sizeof(frame));
    snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 111u, 50u, &freshness);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(freshness.sequence == 2u);
    // CRC 计算失败不刷新上一份接收时间，也不提交任何发送分片。
    crc_fail_after = 0;
    deliver(112u, frame, sizeof(frame));
    TEST_CHECK(comm->freshness.timestamp_ms == 61u && comm->freshness.sequence == 2u);
    crc_fail_after = 0;
    CANCommSend(comm, payload);
    TEST_CHECK(sends == 0u);
    CANCommSend(comm, payload);
    TEST_CHECK(sends == 1u);
    free(comm);
    return 0;
}
