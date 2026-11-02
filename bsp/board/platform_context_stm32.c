/** @file platform_context_stm32.c
 * @brief 将上下文查询映射到 Cortex-M 的异常编号。
 */
#include "../os/platform_irq.h"
#include "main.h"

uint8_t PlatformIrqIsInHandler(void)
{
    return __get_IPSR() != 0u; // 非零表示正在执行中断或异常处理。
}
