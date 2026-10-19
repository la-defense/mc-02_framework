/**
 * @file bsp_can.h
 * @brief LK 主机回归使用的 CAN 替身；保留 H723 三总线及发送头字段。
 */
#ifndef MC02_LK_TEST_CAN_H
#define MC02_LK_TEST_CAN_H
#include <stdint.h>

#define DEVICE_CAN_CNT 3u
#define CAN_MX_REGISTER_CNT 16u
typedef struct { uint8_t bus_index; } FDCAN_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;

typedef struct
{
    uint32_t Identifier;
} LKTestCanHeader_t;

typedef struct CANInstance
{
    FDCAN_HandleTypeDef *can_handle;
    LKTestCanHeader_t txconf;
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t tx_buff[8];
    uint8_t rx_buff[8];
    uint8_t rx_len;
    void (*can_module_callback)(struct CANInstance *);
    void *id;
} CANInstance;

typedef struct
{
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *instance, float timeout);

#endif
