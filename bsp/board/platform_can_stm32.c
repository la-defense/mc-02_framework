/** @file platform_can_stm32.c
 * @brief 将逻辑 CAN 总线队列接口映射到 MC-02 的现有 H723 实现。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>
#include <string.h>

uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus)
{
    return (uint8_t)(BoardCanResolve(bus) != NULL); // 只检查静态绑定，不查询设备状态。
}

static void PlatformCanRxDispatch(CANInstance *instance)
{
    // 底层由接收任务调用此入口；context 保存在调用者长期端点中。
    PlatformCanRxPort_t *port = instance == NULL ? NULL : instance->id;
    if (port == NULL || port->backend != instance || port->callback == NULL) return;
    if (instance->rx_len > sizeof(instance->rx_buff)) return; // 拒绝损坏长度，避免越界复制。
    PlatformCanRxFrame_t frame = {0}; // 局部副本仅在回调期间有效。
    frame.length = instance->rx_len; // 保留实际接收长度。
    frame.received_ms = instance->rx_timestamp_ms; // 保留入队时间，不使用派发时刻。
    memcpy(frame.data, instance->rx_buff, frame.length); // 复制有效字节供上层只读处理。
    port->callback(port->context, &frame); // 同一接收任务中同步派发。
}

uint8_t PlatformCanRxRegister(PlatformCanRxPort_t *port, const PlatformCanEndpoint_t *endpoint,
                              PlatformCanRxCallback_t callback, void *context)
{
    if (port == NULL || port->backend != NULL || endpoint == NULL || callback == NULL ||
        endpoint->tx_id > 0x7ffu || endpoint->rx_id > 0x7ffu) return 0u;
    CAN_Init_Config_s config = {0}; // 先在局部准备配置，失败不改调用者对象。
    if (!BoardCanPrepareEndpoint(endpoint, &config)) return 0u; // 未绑定时不注册资源。
    config.can_module_callback = PlatformCanRxDispatch; // 底层只进入平台适配器。
    config.id = port; // 保存长期端点地址，供派发时找回回调及上下文。
    CANInstance *instance = CANRegister(&config); // 沿用现有过滤器和资源分配。
    if (instance == NULL) return 0u; // 底层失败时端点保持原值。
    port->callback = callback; // 注册成功后保存上层处理函数。
    port->context = context; // 保存上层上下文，接收任务启动前完成。
    port->backend = instance; // 最后发布底层实例，允许后续派发。
    return 1u;
}

void PlatformCanRxRollback(PlatformCanRxPort_t *port)
{
    if (port == NULL || port->backend == NULL) return; // 无注册时不访问底层。
    CANUnregisterInstance((CANInstance *)port->backend); // 启动回退时释放本次注册。
    memset(port, 0, sizeof(*port)); // 释放后清除回调及旧句柄，允许重新配置。
}

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
