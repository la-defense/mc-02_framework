#ifndef MC02_CAN_COMM_TEST_BSP_CAN_H
#define MC02_CAN_COMM_TEST_BSP_CAN_H
#include <stdint.h>
typedef struct { uint8_t rx_buff[8]; uint8_t tx_buff[8]; uint8_t rx_len; uint32_t rx_timestamp_ms; void *id; } CANInstance;
typedef struct { void *can_handle; uint32_t tx_id, rx_id; void (*can_module_callback)(CANInstance *); void *id; } CAN_Init_Config_s;
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, float timeout);
#endif
