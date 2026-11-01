/** @file bsp_can.h
 * @brief 记录 DJI 生产驱动的 CAN 提交，不访问真实总线。
 */
#ifndef MC02_PID_TEST_CAN_H
#define MC02_PID_TEST_CAN_H
#include <stdint.h>
typedef struct { unsigned id; } FDCAN_HandleTypeDef;
typedef FDCAN_HandleTypeDef CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
typedef struct CANInstance
{
    CAN_HandleTypeDef *can_handle;
    struct { uint32_t StdId, IDE, RTR, DLC; } txconf;
    uint32_t tx_id, rx_id;
    uint8_t tx_buff[8], rx_buff[8], rx_len;
    uint32_t rx_timestamp_ms, rx_timestamp_cycles;
    void (*can_module_callback)(struct CANInstance *);
    void *id;
} CANInstance;
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
/** @brief 保存测试实例。
 * @param[in] config 有效配置，测试提供最多四份实例。
 * @return 已登记的测试对象。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);
#include "../can_tx_fake_api.h"
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id);
CAN_TxSubmitStatus_t CANSubmitPeriodicPrepared(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
                                       uint32_t *request_id);
CAN_TxSubmitStatus_t CANSubmitOneShot(FDCAN_HandleTypeDef *hcan, const void *source,
                                      const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
                                      uint32_t deadline_ms, uint32_t *request_id);
#endif
