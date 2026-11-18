/** @file platform_periodic_freertos.c
 * @brief 用 FreeRTOS 的绝对时刻等待任务拍点，处理延迟唤醒和计数回绕。
 */
#include "platform_periodic.h"
#include "platform_irq.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

_Static_assert(sizeof(TickType_t) == sizeof(uint32_t), "Periodic backend needs 32-bit ticks");

static uint8_t TaskContextReady(void)
{
    return (uint8_t)(PlatformIrqIsInHandler() == 0u &&
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING); // 只在任务上下文和正常调度状态下等待。
}

PlatformPeriodicResult_t PlatformTaskPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config)
{
    if (!TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT; // 在等待接口可用后建立计数起点。
    const uint32_t tick_hz = configTICK_RATE_HZ; // 使用后端实际每秒刻度数。
    if (tick_hz == 0u || 1000000u % tick_hz != 0u)
        return PLATFORM_PERIODIC_CONFIG; // 一个刻度无法准确写成整数微秒时拒绝此后端。
    const uint32_t quantum_us = 1000000u / tick_hz; // 换算每刻度的固定时长。
    return PlatformPeriodicInit(clock, config, quantum_us, xTaskGetTickCount()); // 使用同一 RTOS 计数域。
}

PlatformPeriodicResult_t PlatformTaskPeriodicWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    if (clock == NULL || release == NULL) return PLATFORM_PERIODIC_CONFIG; // 输出不可为空。
    if (!TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT; // ISR 和暂停调度的代码不能睡眠。
    PlatformPeriodicClock_t candidate = *clock; // 等待或复查失败时保留原周期状态。
    PlatformPeriodicRelease_t observed; // 先在任务内判断是否已经到达计划时间。
    TickType_t anchor = xTaskGetTickCount(); // 用此刻度同时作为计算与等待起点。
    PlatformPeriodicResult_t status = PlatformPeriodicPoll(&candidate, anchor, &observed); // 已到期时直接领取。
    if (status == PLATFORM_PERIODIC_WAIT)
    {
        vTaskDelayUntil(&anchor, observed.wait_ticks); // 使用绝对时刻，调用前的抢占不增加额外等待。
        status = PlatformPeriodicPoll(&candidate, xTaskGetTickCount(), &observed); // 实际醒来可能已错过若干拍。
        if (status == PLATFORM_PERIODIC_WAIT) return PLATFORM_PERIODIC_CLOCK; // 提前唤醒时不能运行任务计算。
    }
    if (status == PLATFORM_PERIODIC_READY)
    {
        *clock = candidate; // 成功领取后固定到下一拍。
        *release = observed; // 同时交付实际时刻与跳拍数。
    }
    return status;
}
