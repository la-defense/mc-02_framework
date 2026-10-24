/** @file dwt_timeline_thread_test.c
 * @brief 用真实线程验证生产 DWT 时间轴并发读取的单调性与累计一致性。
 * @note 主机模拟器件周期寄存器；不代表芯片计时精度或中断时序。
 */
#include "test_check.h"
#include "bsp_dwt.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>

DWT_Type host_dwt;
CoreDebug_Type host_core_debug;

static atomic_int counter_done;
static atomic_int failed;

static void *AdvanceCounter(void *unused)
{
    (void)unused;
    for (uint32_t index = 0u; index < 10000u; ++index)
        (void)atomic_fetch_add(&DWT->CYCCNT, 48000u);
    atomic_store(&counter_done, 1);
    return NULL;
}

static void *ReadTimeline(void *unused)
{
    (void)unused;
    uint64_t previous = 0u;
    do
    {
        uint64_t current = DWT_GetTimeline_us();
        if (current < previous)
            atomic_store(&failed, 1);
        previous = current;
    } while (!atomic_load(&counter_done));

    for (uint32_t index = 0u; index < 1000u; ++index)
    {
        uint64_t current = DWT_GetTimeline_us();
        if (current < previous)
            atomic_store(&failed, 1);
        previous = current;
    }
    return NULL;
}

int main(void)
{
    DWT_Init(480u);

    pthread_t counter;
    pthread_t readers[2];
    TEST_CHECK(pthread_create(&counter, NULL, AdvanceCounter, NULL) == 0);
    TEST_CHECK(pthread_create(&readers[0], NULL, ReadTimeline, NULL) == 0);
    TEST_CHECK(pthread_create(&readers[1], NULL, ReadTimeline, NULL) == 0);
    TEST_CHECK(pthread_join(counter, NULL) == 0);
    TEST_CHECK(pthread_join(readers[0], NULL) == 0);
    TEST_CHECK(pthread_join(readers[1], NULL) == 0);
    TEST_CHECK(atomic_load(&failed) == 0);

    uint64_t expected_us = atomic_load(&DWT->CYCCNT) / 480u;
    TEST_CHECK(DWT_GetTimeline_us() == expected_us);
    return 0;
}
