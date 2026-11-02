/** @file platform_can_periodic_stm32.c
 * @brief 将逻辑 CAN 周期发送映射到 MC-02 的现有队列。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

CAN_TxSubmitStatus_t PlatformCanSubmitPeriodic(PlatformCanBus_t bus, const void *source,
    uint32_t key, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
    uint32_t *request_id)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 用逻辑编号查询板级绑定。
    if (handle == NULL) return CAN_TX_SUBMIT_NOT_READY; // 总线未绑定时保留输出编号。
    return CANSubmitPeriodicPrepared(handle, source, key, frame, priority, deadline_ms,
                                     period_us, prepare, request_id); // 保留发送前检查和失败回退。
}
