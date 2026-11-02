/** @file platform_can_rx_fake.c
 * @brief 为链路主机测试绑定测试控制器，注册和接收由各测试模型提供。
 */
#include "../../bsp/board/board_can_stm32.h"
#include <stddef.h>

uint8_t BoardCanPrepareEndpoint(const PlatformCanEndpoint_t *endpoint, CAN_Init_Config_s *output)
{
    if (endpoint == NULL || output == NULL) return 0u;
    FDCAN_HandleTypeDef *handle = BoardCanResolve(endpoint->can_bus);
    if (handle == NULL) return 0u;
    const CAN_Init_Config_s candidate = {.can_handle=handle, .tx_id=endpoint->tx_id, .rx_id=endpoint->rx_id};
    *output = candidate;
    return 1u;
}

void CANUnregisterInstance(CANInstance *instance)
{
    (void)instance; // 测试使用静态实例，没有可释放的硬件或内存。
}
