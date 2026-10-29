/** @file bsp_can.h
 * @brief 超级电容主机测试用 CAN 接口，测试进程不访问硬件。
 */
#ifndef SUPER_CAP_TEST_BSP_CAN_H
#define SUPER_CAP_TEST_BSP_CAN_H
#include <stddef.h>
#include <stdint.h>
typedef struct { unsigned unused; } FDCAN_HandleTypeDef;
typedef struct CANInstance CANInstance;
struct CANInstance {
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint8_t rx_buff[8], rx_len;
    uint32_t rx_timestamp_ms;
    void (*can_module_callback)(CANInstance *);
    void *id;
};
typedef struct {
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
#include "../can_tx_fake_api.h"
CANInstance *CANRegister(CAN_Init_Config_s *config);
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
    uint32_t slot, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t period_us, uint32_t *request_id);
#endif
