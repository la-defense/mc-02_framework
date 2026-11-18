/** @file platform_periodic_tim5_stm32.c
 * @brief 用 MC-02 的空闲 TIM5 和专用信号量唤醒一个周期任务。
 */
#include "../os/platform_periodic.h"
#include "../os/platform_irq.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <limits.h>
#include <stddef.h>

#define TIMER_HZ UINT32_C(1000000) // TIM5 计数一次对应 1 μs。

static StaticSemaphore_t signal_storage; // 定时器使用独立的二值信号量，不占应用的通知位。
static SemaphoreHandle_t wake_signal;
static TaskHandle_t owner_task; // 当前拥有资源的任务，申请和释放时修改。
static const PlatformPeriodicClock_t *owner_clock; // 防止同一任务混用多个时钟对象。
volatile uint32_t mc02_tim5_irq_notifications; // 裸板只读观测：本次申请实际产生的有效通知数。

static uint8_t TaskContextReady(void)
{
    return (uint8_t)(!PlatformIrqIsInHandler() && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

// 定时器由本模块独占，清除全部标志不会影响其它外设。
static void DisarmCompare(void)
{
    TIM5->DIER = 0u; // 先停止比较中断的产生。
    TIM5->SR = 0u; // 清除定时器内的旧比较和更新标志。
    __DSB(); // 外设写入完成后才清除 NVIC 挂起位。
    NVIC_ClearPendingIRQ(TIM5_IRQn); // 已送到中断控制器的旧请求也要清掉。
}

// 按当前 MC-02 的 TIMPRE=0 时钟规则计算 1 MHz 分频。
static uint32_t TimerDivider(void)
{
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0u) return 0u; // 不支持的时钟树配置直接拒绝。
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency); // 读取实际 APB1 分频设置。
    const uint32_t pclk = HAL_RCC_GetPCLK1Freq(); // TIM5 位于 APB1 域。
    const uint32_t factor = clocks.APB1CLKDivider == RCC_APB1_DIV1 ? 1u : 2u; // 未分频时同频，否则计数器时钟为两倍 PCLK1。
    if (pclk == 0u || pclk > UINT32_MAX / factor) return 0u; // 防止错误频率及乘法溢出。
    const uint32_t timer_hz = pclk * factor; // 得到分频前的硬件时钟。
    if (timer_hz % TIMER_HZ != 0u || timer_hz / TIMER_HZ > (uint32_t)UINT16_MAX + 1u)
        return 0u; // PSC 是 16 位，且输出刻度必须恰好为 1 μs。
    return timer_hz / TIMER_HZ;
}

PlatformPeriodicResult_t PlatformTaskHighResOpen(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config)
{
    if (!TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT; // 没有正常任务上下文时不能使用信号量等待。
    PlatformPeriodicClock_t validated; // 所有配置检查成功前不改写调用者状态。
    if (clock == NULL || PlatformPeriodicInit(&validated, config, 1u, 0u) != PLATFORM_PERIODIC_READY)
        return PLATFORM_PERIODIC_CONFIG;
    const uint32_t divider = TimerDivider(); // 先检查时钟树，再接管外设。
    if (divider == 0u || HAL_NVIC_GetPriorityGrouping() != NVIC_PRIORITYGROUP_4)
        return PLATFORM_PERIODIC_CONFIG; // 中断优先级必须能调用 FreeRTOS 的 ISR 接口。
    taskENTER_CRITICAL(); // 检查占用与取得资源之间不允许切换任务。
    if (owner_task != NULL || __HAL_RCC_TIM5_IS_CLK_ENABLED() || NVIC_GetEnableIRQ(TIM5_IRQn))
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_RESOURCE; // 他人已经启用的时钟或中断不得清理。
    }
    wake_signal = xSemaphoreCreateBinaryStatic(&signal_storage); // 无需分配堆内存。
    if (wake_signal == NULL)
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_RESOURCE;
    }
    __HAL_RCC_TIM5_CLK_ENABLE(); // 先提供外设寄存器时钟。
    __HAL_RCC_TIM5_FORCE_RESET(); // 清除旧计数模式、通道模式及输出脚设置。
    __HAL_RCC_TIM5_RELEASE_RESET();
    TIM5->PSC = divider - 1u; // 寄存器写入分频值减一。
    TIM5->ARR = UINT32_MAX; // 保留完整的 32 位计数范围。
    TIM5->EGR = TIM_EGR_UG; // 装载预分频和计数初值。
    DisarmCompare(); // 在开放 NVIC 前清掉初始更新标志。
    owner_task = xTaskGetCurrentTaskHandle(); // 记录可等待和释放的唯一任务。
    owner_clock = clock; // 将硬件计数域与该任务的状态对象绑定。
    mc02_tim5_irq_notifications = 0u; // 每次检查单独统计实际唤醒。
    HAL_NVIC_SetPriority(TIM5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0u); // 使用可调用 ISR 信号量接口的优先级。
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 = TIM_CR1_CEN; // 使用默认向上计数和内部时钟，不打开输出通道。
    validated.next_tick = TIM5->CNT; // 第一拍立即到期。
    validated.last_tick = validated.next_tick; // 同一初值用于检查计数前进。
    *clock = validated; // 完成配置后才交付。
    taskEXIT_CRITICAL();
    return PLATFORM_PERIODIC_READY;
}

// 在半个计数范围内判断是否到达目标，允许 CNT 自然回绕。
static uint8_t Reached(uint32_t now, uint32_t deadline)
{
    return (uint8_t)((uint32_t)(now - deadline) <= INT32_MAX);
}

PlatformPeriodicResult_t PlatformTaskHighResWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    if (clock == NULL || release == NULL) return PLATFORM_PERIODIC_CONFIG;
    if (!TaskContextReady() || owner_clock != clock || owner_task != xTaskGetCurrentTaskHandle())
        return PLATFORM_PERIODIC_CONTEXT; // 无权使用时不得读写 TIM5。
    PlatformPeriodicClock_t candidate = *clock; // 等待失败不会移动原拍点。
    PlatformPeriodicRelease_t observed;
    PlatformPeriodicResult_t result = PlatformPeriodicPoll(&candidate, TIM5->CNT, &observed);
    if (result == PLATFORM_PERIODIC_WAIT)
    {
        (void)xSemaphoreTake(wake_signal, 0u); // 清掉上次结束后可能留在信号量中的通知。
        taskENTER_CRITICAL(); // 写入比较值期间禁止本定时器中断介入。
        DisarmCompare();
        TIM5->CCR1 = observed.scheduled_tick; // 通道 1 只比较计数，不驱动引脚。
        uint8_t armed = 0u;
        if (!Reached(TIM5->CNT, observed.scheduled_tick))
        {
            TIM5->DIER = TIM_DIER_CC1IE; // 只允许通道 1 比较中断。
            armed = 1u; // 判定尚未到期；之后到期将由 ISR 保留通知。
        }
        taskEXIT_CRITICAL();

        const uint64_t ticks = ((uint64_t)observed.wait_ticks * configTICK_RATE_HZ + TIMER_HZ - 1u) / TIMER_HZ + 1u; // 向上换算并留一刻度，防止整 tick 边界使超时提前。
        const TickType_t limit = ticks >= portMAX_DELAY ? portMAX_DELAY - 1u : (TickType_t)ticks; // 绝不无限等待丢失的中断。
        const uint8_t signaled = (uint8_t)(!armed || xSemaphoreTake(wake_signal, limit) == pdTRUE); // 已到期的竞态可直接复查。
        taskENTER_CRITICAL();
        DisarmCompare(); // 超时或唤醒后取消本次比较，下一轮不继承中断。
        taskEXIT_CRITICAL();
        if (!signaled) return PLATFORM_PERIODIC_CLOCK; // 计数前进也不能掩盖预期中断丢失。
        result = PlatformPeriodicPoll(&candidate, TIM5->CNT, &observed); // 只接受硬件时刻已经到期的结果。
        if (result == PLATFORM_PERIODIC_WAIT) return PLATFORM_PERIODIC_CLOCK; // 提前通知不能运行新一拍。
    }
    if (result == PLATFORM_PERIODIC_READY)
    {
        *clock = candidate;
        *release = observed;
    }
    return result;
}

PlatformPeriodicResult_t PlatformTaskHighResClose(const PlatformPeriodicClock_t *clock)
{
    if (clock == NULL || !TaskContextReady() || owner_clock != clock ||
        owner_task != xTaskGetCurrentTaskHandle()) return PLATFORM_PERIODIC_CONTEXT;
    taskENTER_CRITICAL(); // 释放过程中不允许新任务接管。
    DisarmCompare();
    HAL_NVIC_DisableIRQ(TIM5_IRQn); // 停止来自 TIM5 的异常请求。
    TIM5->CR1 = 0u; // 停止计数后再关闭外设时钟。
    __HAL_RCC_TIM5_CLK_DISABLE();
    vSemaphoreDelete(wake_signal); // 外设和 IRQ 停止后才释放静态信号量。
    wake_signal = NULL; // 下一次申请重新初始化通知。
    owner_clock = NULL; // 关闭后旧状态地址不再有权限。
    owner_task = NULL; // 下一任务可以重新初始化硬件。
    taskEXIT_CRITICAL();
    return PLATFORM_PERIODIC_READY;
}

void TIM5_IRQHandler(void)
{
    if ((TIM5->SR & TIM_SR_CC1IF) == 0u || (TIM5->DIER & TIM_DIER_CC1IE) == 0u)
        return; // 非本次比较的请求既不通知，也不取消还没到期的比较。
    DisarmCompare(); // 每次设置只产生一次唤醒。
    BaseType_t reschedule = pdFALSE;
    if (xSemaphoreGiveFromISR(wake_signal, &reschedule) == pdTRUE)
        ++mc02_tim5_irq_notifications; // 只记录成功写入信号量的通知。
    portYIELD_FROM_ISR(reschedule); // 如需要，立即交还调度器。
}
