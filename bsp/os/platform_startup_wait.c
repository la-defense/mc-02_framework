/** @file platform_startup_wait.c
 * @brief 使用调用方周期计数和最大检查次数限制启动等待。
 */
#include "platform_startup_wait.h"
#include <stddef.h>

PlatformStartupWaitResult_t PlatformStartupWait(PlatformStartupReadyFn ready,
    PlatformStartupCyclesFn cycles, void *context, uint32_t timeout_cycles, uint32_t max_polls)
{
    // 无效参数不能开始等待，也不能调用输入函数。
    if (ready == NULL || cycles == NULL || timeout_cycles == 0u ||
        timeout_cycles > INT32_MAX || max_polls == 0u)
        return STARTUP_WAIT_ARGUMENT;

    const uint32_t start = cycles(context); // 固定起点，后续检查不能续期。
    for (uint32_t poll = 0u; poll < max_polls; ++poll)
    {
        const uint8_t condition_ready = ready(context); // 每次调用都计入固定期限。
        if ((uint32_t)(cycles(context) - start) >= timeout_cycles)
            return STARTUP_WAIT_TIMEOUT; // 检查函数自身耗时也可能让期限到达。
        if (condition_ready != 0u)
            return STARTUP_WAIT_READY; // 条件检查完成时仍在期限内。
    }
    return STARTUP_WAIT_POLL_LIMIT; // 周期计数停止时也能按最大次数退出。
}
