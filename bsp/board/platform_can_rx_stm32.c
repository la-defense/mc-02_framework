/** @file platform_can_rx_stm32.c
 * @brief 将逻辑 CAN 接收端点映射到 MC-02 的 H723 接收派发。
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
    frame.received_ticks = instance->rx_timestamp_cycles; // 保留同时记录的周期数，供高频反馈计算间隔。
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

uint8_t PlatformCanRxIsRegistered(const PlatformCanRxPort_t *port)
{
    return (uint8_t)(port != NULL && port->backend != NULL);
}

void PlatformCanRxRollback(PlatformCanRxPort_t *port)
{
    if (port == NULL || port->backend == NULL) return; // 无注册时不访问底层。
    CANUnregisterInstance((CANInstance *)port->backend); // 启动回退时释放本次注册。
    memset(port, 0, sizeof(*port)); // 释放后清除回调及旧句柄，允许重新配置。
}
