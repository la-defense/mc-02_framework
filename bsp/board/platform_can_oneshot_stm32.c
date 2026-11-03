/** @file platform_can_oneshot_stm32.c
 * @brief 把逻辑 CAN 一次性命令交给 MC-02 原有发送队列。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

CAN_TxSubmitStatus_t PlatformCanSubmitOneShot(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frame, CAN_TxPriority_t priority, uint32_t deadline_ms, uint32_t *request_id)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 板级表将逻辑编号映射到控制器。
    if (handle == NULL) return CAN_TX_SUBMIT_NOT_READY; // 未绑定时不修改输出编号。
    return CANSubmitOneShot(handle, source, frame, priority, deadline_ms, request_id); // 保留原队列规则。
}
