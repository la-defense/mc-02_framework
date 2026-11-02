/** @file platform_can_stm32.c
 * @brief 将逻辑 CAN 总线队列接口映射到 MC-02 的现有 H723 实现。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

CAN_TxSubmitStatus_t PlatformCanSubmitBatch(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 查询板级绑定。
    if (handle == NULL) return CAN_TX_SUBMIT_NOT_READY; // 未绑定时不触碰输出编号。
    return CANSubmitInterruptibleBatch(handle, source, frames, count, priority, deadline_ms, request_id);
}

CAN_TxSubmitStatus_t PlatformCanSubmitBatchUntil(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 查询板级绑定。
    if (handle == NULL) return CAN_TX_SUBMIT_NOT_READY; // 未绑定时不触碰输出编号。
    return CANSubmitInterruptibleBatchUntil(handle, source, frames, count, priority, expiry_ms, request_id);
}

uint8_t PlatformCanGetRequest(uint32_t request_id, CAN_TxResult_t *result)
{
    return CANGetTxRequestState(request_id, result); // 沿用队列结果及失败时的输出约定。
}

uint8_t PlatformCanCancelRequest(uint32_t request_id)
{
    return CANCancelTxRequest(request_id); // 已交给硬件的帧仍不能撤回。
}

uint8_t PlatformCanProcessTxBus(PlatformCanBus_t bus)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 找到本板控制器。
    if (handle == NULL) return 0u; // 无绑定时不调用底层服务。
    CANProcessTxBus(handle); // 继续只为这一总线提供一次有界推进机会。
    return 1u; // 已调用服务，不代表硬件完成发送。
}
