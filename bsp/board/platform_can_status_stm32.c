/** @file platform_can_status_stm32.c
 * @brief 将逻辑 CAN 状态查询映射到 MC-02 控制器。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

PlatformCanStatus_t PlatformCanGetStatus(PlatformCanBus_t bus)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 只通过板级表查找控制器。
    CAN_Status_t status; // 接收驱动提供的当前状态。
    if (handle == NULL) return PLATFORM_CAN_STATUS_UNKNOWN; // 未绑定时不访问硬件。
    CANGetStatus(handle, &status); // 沿用现有驱动的状态判断。
    switch (status.status)
    {
    case CAN_STATUS_OK: return PLATFORM_CAN_STATUS_OK;
    case CAN_STATUS_ERROR: return PLATFORM_CAN_STATUS_ERROR;
    case CAN_STATUS_BUSOFF: return PLATFORM_CAN_STATUS_BUSOFF;
    default: return PLATFORM_CAN_STATUS_UNKNOWN;
    }
}
