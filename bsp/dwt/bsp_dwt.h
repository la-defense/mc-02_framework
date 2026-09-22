/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @author  modified by NeoZng
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"
#include "bsp_log.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/* -----------------------------------------------------------------------------
   热路径轻量探针(2026-09 新增, 用于 CPU 占用排查)
   - 直接读 CYCCNT, 没有 64bit 除法、没有回绕处理, 适合放进 1kHz 任务和 ISR;
   - 限制: 单次测量区间必须远小于 CYCCNT 回绕周期(480MHz 下 8.94s);
   - DWT_GetTimeline_us() 内部有 64bit 除法, 不要放进热路径当探针。
   -------------------------------------------------------------------------- */
extern volatile uint32_t dwt_cpu_freq_mhz; /* 由 DWT_Init() 设置 */

typedef struct
{
    volatile uint32_t last_us;  /* 最近一次耗时 */
    volatile uint32_t max_us;   /* 最大耗时 */
    volatile uint32_t total_us; /* 累计耗时(4295s 回绕, 会话内足够) */
    volatile uint32_t calls;    /* 调用次数 */
} DWT_Probe_t;

static inline uint32_t DWT_ProbeStart(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t DWT_ProbeElapsedCycles(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start);
}

static inline uint32_t DWT_ProbeElapsedUs(uint32_t start)
{
    uint32_t f = dwt_cpu_freq_mhz;
    if (f == 0u)
        f = 480u;
    return DWT_ProbeElapsedCycles(start) / f;
}

static inline void DWT_ProbeDone(DWT_Probe_t *probe, uint32_t start)
{
    uint32_t dt = DWT_ProbeElapsedUs(start);
    probe->last_us = dt;
    if (dt > probe->max_us)
        probe->max_us = dt;
    probe->total_us += dt;
    probe->calls++;
}

/**
 * @brief 该宏用于计算代码段执行时间,单位为秒/s,返回值为float类型
 *        首先需要创建一个float类型的变量,用于存储时间间隔
 *        计算得到的时间间隔同时还会通过RTT打印到日志终端,你也可以将你的dt变量添加到查看
 */
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = DWT_GetTimeline_s();      \
        code;                                    \
        dt = DWT_GetTimeline_s() - tstart;       \
        /* RTT 的 printf 不支持 %f 且遇到 %f 不消费参数 → 会打崩后面的 %s, */ \
        /* 所以放大成整数打印(单位 us)。 */                                \
        LOGINFO("[DWT] " #dt " = %ld us\r\n", (long)(dt * 1000000.0f));    \
    } while (0)

/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 *
 * @param CPU_Freq_mHz c板为168MHz,A板为180MHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间轴
 */
float DWT_GetTimeline_s(void);

/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 *
 * @return float
 */
float DWT_GetTimeline_ms(void);

/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief DWT延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为秒/s
 */
void DWT_Delay(float Delay);

/**
 * @brief DWT更新时间轴函数,会被三个timeline函数调用
 * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void DWT_SysTimeUpdate(void);

#endif /* BSP_DWT_H_ */
