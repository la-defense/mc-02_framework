/** @file platform_can_request_fake.c
 * @brief 将对时测试的平台请求查询接到该测试已有的 CAN 队列模型。
 */
#include "bsp_can.h"
#include "../../bsp/can/platform_can.h"

CAN_TxSubmitStatus_t PlatformCanSubmitBatch(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    static FDCAN_HandleTypeDef test_handle; // 对时测试只依赖设备身份存在，不访问硬件。
    if (bus != PLATFORM_CAN_BUS_1 && bus != PLATFORM_CAN_BUS_2 && bus != PLATFORM_CAN_BUS_3)
        return CAN_TX_SUBMIT_NOT_READY;
    return CANSubmitInterruptibleBatch(&test_handle, source, frames, count, priority, deadline_ms, request_id);
}

uint8_t PlatformCanGetRequest(uint32_t request_id, CAN_TxResult_t *result)
{
    return CANGetTxRequestState(request_id, result); // 沿用测试进程控制的请求状态。
}

uint8_t PlatformCanCancelRequest(uint32_t request_id)
{
    return CANCancelTxRequest(request_id); // 沿用测试进程记录的取消行为。
}
