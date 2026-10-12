#include "test_check.h"
#include "can_comm.h"
#include <stdlib.h>
#include <string.h>

static uint32_t now_ms;
static uint8_t daemon_online;
static CANInstance can_instance;
static DaemonInstance daemon_instance;
static void (*rx_callback)(CANInstance *);

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config) { (void)config; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; daemon_online = 1u; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return daemon_online; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    can_instance.id = config->id;
    rx_callback = config->can_module_callback;
    return &can_instance;
}
void CANSetDLC(CANInstance *instance, uint8_t length) { (void)instance; (void)length; }
uint8_t CANTransmit(CANInstance *instance, float timeout) { (void)instance; (void)timeout; return 1u; }
uint8_t crc_8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; ++i) crc = (uint8_t)(crc + data[i]);
    return crc;
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
    CANComm_Init_Config_s config = {.send_data_len = 4u, .recv_data_len = 4u, .daemon_count = 5u};
    CANCommInstance *comm = CANCommInit(&config);
    TEST_CHECK(comm != NULL && rx_callback != NULL);
    uint8_t payload[4] = {1u, 2u, 3u, 4u};
    uint8_t frame[8] = {CAN_COMM_HEADER, 4u, 1u, 2u, 3u, 4u, 10u, CAN_COMM_TAIL};
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
    frame[6] = 10u;
    deliver(61u, frame, sizeof(frame));
    snapshot_valid = CANCommGetSnapshot(comm, output, sizeof(output), 111u, 50u, &freshness);
    TEST_CHECK(snapshot_valid == 1u);
    TEST_CHECK(freshness.sequence == 2u);
    free(comm);
    return 0;
}
