/** @file bsp_can.h
 * @brief HT04主机测试的CAN登记与发送入口；只复制数据，不访问总线。
 */
#ifndef MC02_HT_NUMERIC_CAN_H
#define MC02_HT_NUMERIC_CAN_H
#include <stdint.h>
typedef struct { unsigned unused; } FDCAN_HandleTypeDef;
typedef FDCAN_HandleTypeDef CAN_HandleTypeDef;
typedef struct CANInstance
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id,rx_id;
    uint8_t tx_buff[8],rx_buff[8];
    uint8_t rx_len;
    uint32_t rx_timestamp_ms;
    uint32_t rx_timestamp_cycles;
    void *id;
} CANInstance;
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id,rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
/** @brief 保存一份测试配置，最多登记两个实例。
 * @param[in] config 非空配置，回调由测试在构帧后调用。
 * @return 测试中的长期存储对象。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);
#include "../can_tx_fake_api.h"
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id);
CAN_TxSubmitStatus_t CANSubmitOneShot(FDCAN_HandleTypeDef *hcan, const void *source,
                                      const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
                                      uint32_t deadline_ms, uint32_t *request_id);
#endif
