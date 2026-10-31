#ifndef MC02_CAN_COMM_TEST_BSP_CAN_H
#define MC02_CAN_COMM_TEST_BSP_CAN_H
#include <stdint.h>
typedef struct { uint32_t unused; } FDCAN_HandleTypeDef;
typedef struct { FDCAN_HandleTypeDef *can_handle; uint8_t rx_buff[8]; uint8_t tx_buff[8]; uint8_t rx_len; uint32_t rx_timestamp_ms; uint32_t tx_id; void *id; } CANInstance;
typedef struct { FDCAN_HandleTypeDef *can_handle; uint32_t tx_id, rx_id; void (*can_module_callback)(CANInstance *); void *id; } CAN_Init_Config_s;
#include "../can_tx_fake_api.h"
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANUnregisterInstance(CANInstance *instance);
void CANProcessTxBus(FDCAN_HandleTypeDef *handle);
CAN_TxSubmitStatus_t CANSubmitOneShotBatch(FDCAN_HandleTypeDef *hcan, const void *source,
                                           const CAN_TxFrame_t *frames, size_t frame_count,
                                           CAN_TxPriority_t priority, uint32_t deadline_ms,
                                           uint32_t *request_id);
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id);
uint8_t CANGetTxRequestState(uint32_t request_id, CAN_TxResult_t *result);
uint8_t CANCancelTxRequest(uint32_t request_id);
#endif
