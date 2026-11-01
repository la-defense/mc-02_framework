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
#ifdef FDCAN
    case PLATFORM_CAN_BUS_3: return &hcan3; // 第三条逻辑总线使用 FDCAN3。
#endif
    default: return NULL; // 未声明的编号不映射到任何设备。
    }
}

uint8_t BoardCanPrepareEndpoint(const PlatformCanEndpoint_t *endpoint, CAN_Init_Config_s *output)
{
    if (endpoint == NULL || output == NULL) return 0u; // 不读取空输入，也不改写输出。
    FDCAN_HandleTypeDef *handle = BoardCanResolve(endpoint->can_bus); // 解析当前板的逻辑总线。
    if (handle == NULL) return 0u; // 未绑定总线时保留调用者的旧配置。
    const CAN_Init_Config_s candidate = {
        .can_handle = handle, // 使用板级表绑定的控制器。
        .tx_id = endpoint->tx_id, // 保留车型指定的发送号或电机编号。
        .rx_id = endpoint->rx_id, // 保留车型指定的接收号。
        .can_module_callback = NULL, // 注册前由驱动填入协议解析函数。
        .id = NULL, // 注册前由驱动填入实例所有者。
    };
    *output = candidate; // 全部转换成功后一次性交付结果。
    return 1u;
}
