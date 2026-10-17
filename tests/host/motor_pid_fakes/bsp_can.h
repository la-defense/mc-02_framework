/** @file bsp_can.h
 * @brief 记录 DJI 生产驱动的 CAN 提交，不访问真实总线。
 */
#ifndef MC02_PID_TEST_CAN_H
#define MC02_PID_TEST_CAN_H
#include <stdint.h>
typedef struct { unsigned id; } CAN_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
typedef struct CANInstance
{
    CAN_HandleTypeDef *can_handle;
    struct { uint32_t StdId, IDE, RTR, DLC; } txconf;
    uint32_t tx_id, rx_id;
    uint8_t tx_buff[8], rx_buff[8];
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
/** @brief 复制提交字节用于检查，不发送 CAN。
 * @param[in] instance 有效发送对象。
 * @param[in] timeout_ms 保留生产参数，替身不等待。
 * @return 固定为1，仅表示替身完成复制。
 */
uint8_t CANTransmit(CANInstance *instance, float timeout_ms);
#endif
