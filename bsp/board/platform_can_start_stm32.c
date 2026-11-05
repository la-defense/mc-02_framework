/** @file platform_can_start_stm32.c
 * @brief 将逻辑 CAN 总线的启动请求交给 H723 后端。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

uint8_t PlatformCanStartBus(PlatformCanBus_t bus)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 查找此逻辑总线绑定的控制器。
    if (handle == NULL) return 0u; // 没有绑定时不启动任何硬件。
    return CANStartBus(handle); // 使用底层按总线独立的启动与失败处理。
}
