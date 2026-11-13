/**
 * @file bsp_dwt.h
 * @brief 生产 CAN 集成回归使用的可控时间与耗时记录替身。
 */
#ifndef MC02_LK_CAN_TEST_DWT_H
#define MC02_LK_CAN_TEST_DWT_H

#include <stddef.h>
#include <stdint.h>
#include "../../../bsp/dwt/platform_probe.h"

typedef PlatformProbe_t DWT_Probe_t;

extern volatile uint32_t dwt_cpu_freq_mhz;
extern volatile uint32_t test_dwt_cycles;
uint64_t DWT_GetTimeline_us(void);
float DWT_GetDeltaT(uint32_t *counter);
#if defined(MC02_CAN_SELFTEST_FAKE_HOOK)
void CANTestProbeHook(void); // 仅集成测试用该钩子模拟回环接收与发送完成中断。
static inline uint32_t DWT_ProbeStart(void) { CANTestProbeHook(); return test_dwt_cycles; }
#else
static inline uint32_t DWT_ProbeStart(void) { return test_dwt_cycles; }
#endif
static inline uint32_t DWT_ProbeElapsedUs(uint32_t start) { (void)start; return 1u; }
static inline void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start)
{
    (void)start;
    if (probe != NULL)
    {
        probe->last_us = 1u;
        if (probe->last_us > probe->max_us)
            probe->max_us = probe->last_us;
        probe->total_us++;
        probe->calls++;
    }
}

#endif
