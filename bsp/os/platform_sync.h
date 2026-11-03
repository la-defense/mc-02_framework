/** @file platform_sync.h
 * @brief 任务临界区和现有毫秒计时接口；不暴露芯片或 RTOS 类型。
 */
#ifndef MC02_PLATFORM_SYNC_H
#define MC02_PLATFORM_SYNC_H
#include <stdint.h>
#include <stdatomic.h>

/** @brief 进入任务临界区，允许按后端规则嵌套。
 * @note 可在调度器启动前或任务上下文调用，不得从中断调用；区内不得等待或调用可能阻塞的接口。
 * 每次进入都要在同一执行上下文配对退出；不替代保存中断屏蔽状态的接口。
 */
void PlatformTaskCriticalEnter(void);

/** @brief 退出一次当前任务的临界区。
 * @note 与 PlatformTaskCriticalEnter 在同一执行上下文按嵌套顺序配对使用。
 */
void PlatformTaskCriticalExit(void);

/** @brief 读取当前平台沿用的毫秒计数。
 * @return 32 位毫秒计数，到 UINT32_MAX 后按无符号规则回绕。
 * @note 当前 H723 后端返回 HAL_GetTick；不改变现有时间基准或提供跨板同步。
 */
uint32_t PlatformTimeNowMs(void);

/** @brief 读取平台已有的 64 位微秒运行时间。
 * @return 自既有时基初始化后累计的微秒值。
 * @note H723 后端沿用 DWT_GetTimeline_us 的计数与同步规则；不提供跨设备对时。
 */
uint64_t PlatformTimeNowUs(void);

/** @brief 发布或读取跨执行上下文共享的普通内存前，建立完整内存顺序。
 * @note 使用 C11 顺序一致栅栏；当前 Cortex-M7 编译器会生成所需的内存屏障指令。
 */
static inline void PlatformMemoryBarrier(void)
{
    atomic_thread_fence(memory_order_seq_cst);
}
#endif
