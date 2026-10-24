/** @file host_critical_model.c
 * @brief 为主机并发回归提供可嵌套的中断屏蔽互斥模型。
 * @note 仅供 WSL pthread 测试使用，不参与固件构建。
 */
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

static pthread_mutex_t host_irq_mutex = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local uint32_t host_irq_depth;

uint32_t HostGetPrimask(void)
{
    return host_irq_depth;
}

void HostDisableIrq(void)
{
    if (host_irq_depth == 0u)
        (void)pthread_mutex_lock(&host_irq_mutex);
    ++host_irq_depth;
}

void HostSetPrimask(uint32_t mask)
{
    if (mask < host_irq_depth)
    {
        host_irq_depth = mask;
        if (host_irq_depth == 0u)
        {
            (void)pthread_mutex_unlock(&host_irq_mutex);
        }
    }
}

void HostTaskCriticalEnter(void) { HostDisableIrq(); }
void HostTaskCriticalExit(void)
{ if (host_irq_depth != 0u) HostSetPrimask(host_irq_depth - 1u); }

void HostDataMemoryBarrier(void)
{
    atomic_thread_fence(memory_order_seq_cst);
}
