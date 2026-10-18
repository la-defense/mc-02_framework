/** @file bsp_can.h
 * @brief HT04主机测试的CAN登记与发送入口；只复制数据，不访问总线。
 */
#ifndef MC02_HT_NUMERIC_CAN_H
#define MC02_HT_NUMERIC_CAN_H
#include <stdint.h>
typedef struct { unsigned unused; } CAN_HandleTypeDef;
typedef struct CANInstance
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id,rx_id;
    uint8_t tx_buff[8],rx_buff[8];
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
/** @brief 记录驱动的发送调用，不执行物理发送。
 * @param[in] instance 非空测试对象。
 * @param[in] timeout_ms 保留生产参数，替身不等待。
 * @return 固定返回1。
 */
uint8_t CANTransmit(CANInstance *instance,float timeout_ms);
#endif
