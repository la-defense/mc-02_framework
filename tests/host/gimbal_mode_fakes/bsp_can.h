/** @file bsp_can.h
 * @brief 为生产云台模式测试提供电机初始化配置和 CAN 句柄声明。
 * @note 测试只记录电机控制请求，不连接或发送真实 CAN 帧。
 */
#ifndef MC02_GIMBAL_MODE_TEST_BSP_CAN_H
#define MC02_GIMBAL_MODE_TEST_BSP_CAN_H

#include <stdint.h>

typedef struct
{
    uint32_t instance;
} FDCAN_HandleTypeDef;

typedef FDCAN_HandleTypeDef CAN_HandleTypeDef;

typedef struct CANInstance
{
    CAN_HandleTypeDef *can_handle;
} CANInstance;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *instance);
    void *id;
} CAN_Init_Config_s;

extern CAN_HandleTypeDef hcan3;

#endif
