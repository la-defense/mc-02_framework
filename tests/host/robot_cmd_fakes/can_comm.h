#ifndef MC02_HOST_FAKE_CAN_COMM_H
#define MC02_HOST_FAKE_CAN_COMM_H
#include <stdint.h>
#include "robot_def.h"
#include "message_freshness.h"
typedef struct { uint8_t unused; } CANCommInstance;
typedef struct {
    struct { CAN_HandleTypeDef *can_handle; uint16_t tx_id, rx_id; } can_config;
    uint8_t send_data_len, recv_data_len;
} CANComm_Init_Config_s;
CANCommInstance *CANCommInit(CANComm_Init_Config_s *config);
uint8_t CANCommGetSnapshot(CANCommInstance *instance, void *data, uint8_t data_len,
                           uint32_t now_ms, uint32_t max_age_ms, MessageFreshness_t *freshness);
void CANCommSend(CANCommInstance *instance, uint8_t *data);
#endif
