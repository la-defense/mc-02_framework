/** @file board_can_stm32.c
 * @brief MC-02 逻辑 CAN 编号与当前 H723 控制器的静态绑定。
 */
#include "board_can_stm32.h"
#include "bsp_can.h" // 沿用现有 hcan 与 H723 FDCAN 句柄名称映射。
#include <stddef.h>

FDCAN_HandleTypeDef *BoardCanResolve(PlatformCanBus_t bus)
{
    switch (bus)
    {
    case PLATFORM_CAN_BUS_1: return &hcan1; // 第一条逻辑总线使用 FDCAN1。
    case PLATFORM_CAN_BUS_2: return &hcan2; // 第二条逻辑总线使用 FDCAN2。
    case PLATFORM_CAN_BUS_3: return &hcan3; // 第三条逻辑总线使用 FDCAN3。
    default: return NULL; // 未声明的编号不映射到任何设备。
    }
}
