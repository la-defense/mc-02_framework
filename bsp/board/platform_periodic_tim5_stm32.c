/** @file platform_periodic_tim5_stm32.c
 * @brief 用 MC-02 的 TIM5 比较通道和独立信号量唤醒周期任务。
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

typedef struct {
    volatile uint32_t *compare; // 硬件拍点写入对应的 CCR 寄存器。
    uint32_t enable; // DIER 中允许本通道中断的位。
    uint32_t flag; // SR 中报告本通道比较到期的位。
} TimerChannel_t;

typedef struct {
    TaskHandle_t task; // 只有申请任务可以等待或释放。
    const PlatformPeriodicClock_t *clock; // 同时核对申请时绑定的周期状态地址。
    StaticSemaphore_t storage; // 每个通道独立存储一次尚未消费的唤醒。
    SemaphoreHandle_t signal; // 有通道归属时，ISR 用此句柄交付唤醒。
} TimerOwner_t;

static const TimerChannel_t channels[] = {
    {&TIM5->CCR1, TIM_DIER_CC1IE, TIM_SR_CC1IF}, // 通道 1 的拍点、中断开关和到期位。
    {&TIM5->CCR2, TIM_DIER_CC2IE, TIM_SR_CC2IF}, // 通道 2 的拍点、中断开关和到期位。
    {&TIM5->CCR3, TIM_DIER_CC3IE, TIM_SR_CC3IF}, // 通道 3 的拍点、中断开关和到期位。
    {&TIM5->CCR4, TIM_DIER_CC4IE, TIM_SR_CC4IF}, // 通道 4 的拍点、中断开关和到期位。
};
#define CHANNEL_COUNT (sizeof(channels) / sizeof(channels[0])) // 比较通道数量决定同时申请的上限。
static TimerOwner_t owners[CHANNEL_COUNT];
static size_t owner_count; // 第一个任务启动计数，最后一个任务关闭计数。
volatile uint32_t mc02_tim5_irq_notifications; // 裸板只读观测：本次计数器启用期间交付的唤醒数。

static uint8_t TaskContextReady(void)
{
    return (uint8_t)(!PlatformIrqIsInHandler() && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
}

// 只有首次启动或最后一次释放时，才清理定时器的全部中断。
static void ClearTimerInterrupts(void)
{
    TIM5->DIER = 0u; // 先停止比较中断的产生。
    WRITE_REG(TIM5->SR, 0u); // 清除定时器内的旧比较和更新标志。
    __DSB(); // 外设写入完成后才清除 NVIC 挂起位。
    NVIC_ClearPendingIRQ(TIM5_IRQn); // 已送到中断控制器的旧请求也要清掉。
}

// SR 写零清除、写一保留；只清本通道，即使其他通道刚好到期也能保住标志。
static void ClearMatch(const TimerChannel_t *channel)
{
    WRITE_REG(TIM5->SR, ~channel->flag);
    __DSB(); // 等外设收到写入后，再检查时刻或退出中断。
}

// 调用者在任务临界区或 TIM5 ISR 中，只撤销指定通道。
static void DisarmCompare(const TimerChannel_t *channel)
{
    TIM5->DIER &= ~channel->enable; // 保留其他任务已经打开的比较中断。
    ClearMatch(channel); // 共享 NVIC 挂起位由其他通道继续使用。
}

// 在任务临界区查找匹配项；找不到时不访问定时器寄存器。
static size_t OwnerChannel(const PlatformPeriodicClock_t *clock)
{
    const TaskHandle_t task = xTaskGetCurrentTaskHandle();
    for (size_t i = 0u; i < CHANNEL_COUNT; ++i)
        if (owners[i].clock == clock && owners[i].task == task) return i;
    return CHANNEL_COUNT;
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

// 调用前检验分频和中断分组；运行时申请需要在任务临界区内使用。
static uint32_t AvailableChannels(uint32_t divider)
{
    const uint8_t clock_enabled = (uint8_t)(__HAL_RCC_TIM5_IS_CLK_ENABLED() != 0u); // 核对硬件时钟。
    const uint8_t irq_enabled = (uint8_t)(NVIC_GetEnableIRQ(TIM5_IRQn) != 0u); // 核对硬件中断。
    if (owner_count == 0u) return clock_enabled || irq_enabled ? 0u : CHANNEL_COUNT; // 已启用的外部资源不可接管。
    if (!clock_enabled || !irq_enabled || TIM5->PSC != divider - 1u || (TIM5->CR1 & TIM_CR1_CEN) == 0u)
        return 0u; // 硬件计数条件已变化，不能接受新的使用者。
    return (uint32_t)(CHANNEL_COUNT - owner_count); // 共享计数器运行正常时只报告还可分配的通道。
}

uint32_t PlatformTaskHighResAvailableChannels(void)
{
    const uint32_t divider = TimerDivider(); // 确认板级时钟能提供 1 μs 刻度。
    if (divider == 0u || HAL_NVIC_GetPriorityGrouping() != NVIC_PRIORITYGROUP_4)
        return 0u; // ISR 优先级必须符合 FreeRTOS 要求。
    return AvailableChannels(divider);
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
    if (AvailableChannels(divider) == 0u)
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_RESOURCE; // 硬件被占用、计数条件已改变或通道已用完。
    }
    const TaskHandle_t task = xTaskGetCurrentTaskHandle(); // 用实际任务身份登记所有权。
    size_t index = CHANNEL_COUNT; // 用越界索引表示尚无可用通道。
    for (size_t i = 0u; i < CHANNEL_COUNT; ++i) // 保留第一个空位，同时检查全部现有申请。
    {
        if (owners[i].task == task || owners[i].clock == clock) // 防止重复绑定任务或覆盖他人的周期状态。
        {
            taskEXIT_CRITICAL();
            return PLATFORM_PERIODIC_RESOURCE;
        }
        if (index == CHANNEL_COUNT && owners[i].task == NULL) index = i; // 优先复用低号空闲通道。
    }
    const uint8_t first = (uint8_t)(owner_count == 0u); // 只在首个申请进入时接管硬件。
    TimerOwner_t *owner = &owners[index]; // 只修改此次取得的通道。
    owner->signal = xSemaphoreCreateBinaryStatic(&owner->storage); // 无需分配堆内存。
    if (owner->signal == NULL) // 申请失败时保留所有已有通道。
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_RESOURCE;
    }
    if (first) // 后续任务共享当前计数值，不能让已有任务的计划时刻跳变。
    {
        __HAL_RCC_TIM5_CLK_ENABLE(); // 先提供外设寄存器时钟。
        __HAL_RCC_TIM5_FORCE_RESET(); // 清除旧计数、通道和 GPIO 输出模式。
        __HAL_RCC_TIM5_RELEASE_RESET(); // 解除复位后才能配置计数器。
        TIM5->PSC = divider - 1u; // 寄存器写入分频值减一。
        TIM5->ARR = UINT32_MAX; // 保留完整的 32 位计数范围。
        TIM5->EGR = TIM_EGR_UG; // 装载预分频和计数初值。
        ClearTimerInterrupts(); // 在开放 NVIC 前清掉初始更新标志。
        mc02_tim5_irq_notifications = 0u; // 从这次硬件启动开始统计唤醒。
        HAL_NVIC_SetPriority(TIM5_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0u); // 允许 ISR 使用 FreeRTOS 信号量。
        HAL_NVIC_EnableIRQ(TIM5_IRQn); // 允许已登记的比较通道发出请求。
        TIM5->CR1 = TIM_CR1_CEN; // 启动内部向上计数，输出通道保持关闭。
    }
    owner->task = task; // 绑定能够等待或释放的任务。
    owner->clock = clock; // 保存每次操作需要核对的状态地址。
    ++owner_count; // 其他持有者仍然存在时不得关闭定时器。
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

// 已取得通道后，任务在临界区内核对运行所需的硬件状态。
static uint8_t TimerCanWake(void)
{
    return (uint8_t)(__HAL_RCC_TIM5_IS_CLK_ENABLED() && // 时钟门关闭时不能读取外设寄存器。
        (TIM5->CR1 & TIM_CR1_CEN) != 0u && // 停转的计数不能用来领取拍点。
        NVIC_GetEnableIRQ(TIM5_IRQn) != 0u); // 比较中断必须能到达任务。
}

PlatformPeriodicResult_t PlatformTaskHighResWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    if (clock == NULL || release == NULL) return PLATFORM_PERIODIC_CONFIG;
    if (!TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT;
    taskENTER_CRITICAL(); // 查找时不与其他任务的申请或释放交错。
    const size_t index = OwnerChannel(clock);
    if (index == CHANNEL_COUNT)
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_CONTEXT; // 无权使用时不得读写 TIM5。
    }
    if (!TimerCanWake())
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_CLOCK; // 计数停止时，即使旧计数已达到拍点也拒绝领取。
    }
    const uint32_t initial_tick = TIM5->CNT; // 在确认可用的同一临界区读取。
    taskEXIT_CRITICAL();
    const TimerChannel_t *channel = &channels[index]; // 该通道只允许当前任务释放。
    const SemaphoreHandle_t signal = owners[index].signal; // 任务等待和 ISR 交付用同一个信号量。
    PlatformPeriodicClock_t candidate = *clock; // 等待失败不会移动原拍点。
    PlatformPeriodicRelease_t observed;
    PlatformPeriodicResult_t result = PlatformPeriodicPoll(&candidate, initial_tick, &observed); // 按刚取得的可用时刻判断。
    if (result == PLATFORM_PERIODIC_WAIT)
    {
        (void)xSemaphoreTake(signal, 0u); // 清掉上次结束后可能留在信号量中的通知。
        taskENTER_CRITICAL(); // 写入比较值期间禁止本定时器中断介入。
        if (!TimerCanWake())
        {
            taskEXIT_CRITICAL();
            return PLATFORM_PERIODIC_CLOCK; // 读取计数后失去定时器时，不再打开本次比较中断。
        }
        DisarmCompare(channel); // 清掉本通道旧比较，保留其他任务的等待。
        *channel->compare = observed.scheduled_tick; // 通道只比较计数，不驱动引脚。
        ClearMatch(channel); // 去掉改写 CCR 前可能刚到期的旧比较；下面用 CNT 复查新拍点。
        uint8_t armed = 0u;
        if (!Reached(TIM5->CNT, observed.scheduled_tick))
        {
            TIM5->DIER |= channel->enable; // 只新增本通道的中断允许位。
            armed = 1u; // 判定尚未到期；之后到期将由 ISR 保留通知。
        }
        taskEXIT_CRITICAL();

        const uint64_t ticks = ((uint64_t)observed.wait_ticks * configTICK_RATE_HZ + TIMER_HZ - 1u) / TIMER_HZ + 1u; // 向上换算并留一刻度，防止整 tick 边界使超时提前。
        const TickType_t limit = ticks >= portMAX_DELAY ? portMAX_DELAY - 1u : (TickType_t)ticks; // 绝不无限等待丢失的中断。
        const uint8_t signaled = (uint8_t)(!armed || xSemaphoreTake(signal, limit) == pdTRUE); // 已到期的竞态可直接复查。
        taskENTER_CRITICAL(); // 防止清比较时中断再次向本任务发通知。
        if (__HAL_RCC_TIM5_IS_CLK_ENABLED())
            DisarmCompare(channel); // 外设寄存器仍可访问时，只取消本通道的比较。
        if (!signaled || !TimerCanWake())
        {
            taskEXIT_CRITICAL();
            return PLATFORM_PERIODIC_CLOCK; // 通知丢失或恢复运行前时钟已失效，拒绝推进周期。
        }
        const uint32_t wake_tick = TIM5->CNT; // 定时器仍可用时才读取唤醒计数。
        taskEXIT_CRITICAL(); // 允许其他已到期通道继续交付通知。
        result = PlatformPeriodicPoll(&candidate, wake_tick, &observed); // 只接受硬件时刻已经到期的结果。
        if (result == PLATFORM_PERIODIC_WAIT) return PLATFORM_PERIODIC_CLOCK; // 提前通知不能运行新一拍。
    }
    if (result == PLATFORM_PERIODIC_READY)
    {
        *clock = candidate;
        *release = observed;
    }
    return result;
}

PlatformPeriodicResult_t PlatformTaskHighResFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, PlatformPeriodicTiming_t *timing)
{
    if (clock == NULL || release == NULL || timing == NULL) return PLATFORM_PERIODIC_CONFIG;
    if (!TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT;
    taskENTER_CRITICAL(); // 在同一短临界区核对任务所有权并取得完成时刻。
    if (OwnerChannel(clock) == CHANNEL_COUNT)
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_CONTEXT; // 未持有通道时不访问硬件寄存器。
    }
    if (!TimerCanWake())
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_CLOCK; // 外设或比较中断失效时不能用残留计数报告完成。
    }
    const uint32_t now = TIM5->CNT; // 在与拍点相同的计数域内读取完成时刻。
    taskEXIT_CRITICAL();
    return PlatformPeriodicFinish(clock, release, now, timing); // 复用回绕、预算和截止时间计算。
}

PlatformPeriodicResult_t PlatformTaskHighResClose(const PlatformPeriodicClock_t *clock)
{
    if (clock == NULL || !TaskContextReady()) return PLATFORM_PERIODIC_CONTEXT;
    taskENTER_CRITICAL(); // 释放过程中不允许新任务接管。
    const size_t index = OwnerChannel(clock); // 只有原任务和原周期状态能关闭此通道。
    if (index == CHANNEL_COUNT)
    {
        taskEXIT_CRITICAL();
        return PLATFORM_PERIODIC_CONTEXT;
    }
    TimerOwner_t *owner = &owners[index]; // 只释放本任务的通道。
    const uint8_t register_accessible = (uint8_t)(__HAL_RCC_TIM5_IS_CLK_ENABLED() != 0u); // 时钟门关闭时只清理软件与 NVIC。
    if (register_accessible)
        DisarmCompare(&channels[index]); // 停止本通道通知后才删除信号量。
    if (--owner_count == 0u) // 最后一个任务离开时才关闭共享硬件。
    {
        HAL_NVIC_DisableIRQ(TIM5_IRQn); // 先停止来自 TIM5 的异常入口。
        if (register_accessible)
        {
            TIM5->CR1 = 0u; // 停止计数，不再产生新标志。
            ClearTimerInterrupts(); // 清掉留在定时器和 NVIC 内的旧请求。
            __HAL_RCC_TIM5_CLK_DISABLE(); // 无后续寄存器访问时再关时钟。
        }
        else
            NVIC_ClearPendingIRQ(TIM5_IRQn); // 时钟已关闭，下一次申请时复位外设。
    }
    vSemaphoreDelete(owner->signal); // 本通道已经关闭，ISR 不会再访问它。
    owner->signal = NULL; // 下次分配时重新初始化信号量。
    owner->clock = NULL; // 撤销旧周期状态的访问权。
    owner->task = NULL; // 通道可由另一任务接管。
    taskEXIT_CRITICAL();
    return PLATFORM_PERIODIC_READY;
}

void TIM5_IRQHandler(void)
{
    if (!__HAL_RCC_TIM5_IS_CLK_ENABLED()) return; // 时钟关闭时不能读取挂起的外设标志。
    const uint32_t pending = TIM5->SR; // 记录进入时已经到期的通道，稍后到期的由下次 IRQ 处理。
    const uint32_t enabled = TIM5->DIER; // 忽略尚未等待的通道留下的比较标志。
    BaseType_t reschedule = pdFALSE; // 任一通知唤醒更高优先级任务时，退出前请求调度。
    for (size_t i = 0u; i < CHANNEL_COUNT; ++i) // 一次中断可能同时服务多个任务。
    {
        const TimerChannel_t *channel = &channels[i]; // 使用同一通道的开关、标志和信号量。
        if ((pending & channel->flag) == 0u || (enabled & channel->enable) == 0u) continue; // 未打开或未到期时不交付。
        DisarmCompare(channel); // 每次设置只产生一次通知，保留其他通道的状态位。
        if (owners[i].signal == NULL) continue; // 时钟恢复后，已释放的通道可能仍残留比较请求。
        if (xSemaphoreGiveFromISR(owners[i].signal, &reschedule) == pdTRUE)
            ++mc02_tim5_irq_notifications; // 只记录成功写入信号量的通知。
    }
    if (reschedule != pdFALSE) portYIELD_FROM_ISR(reschedule); // 全部交付后只请求一次调度。
}
