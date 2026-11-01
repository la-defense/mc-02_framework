/** @file platform_sync_freertos.c
 * @brief 将框架任务临界区和毫秒计数映射到 H723／FreeRTOS。
 */
#include "platform_sync.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void PlatformTaskCriticalEnter(void)
{
    taskENTER_CRITICAL(); // 使用现有 FreeRTOS 嵌套规则，保护当前任务的共享数据。
}

void PlatformTaskCriticalExit(void)
{
    taskEXIT_CRITICAL(); // 恢复同一任务最近一次进入的临界区。
}

uint32_t PlatformTimeNowMs(void)
{
    return HAL_GetTick(); // 保持原有计数来源和 32 位回绕行为。
}
