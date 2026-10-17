/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @author  modified by NeoZng
 * @version V1.2.0
 * @date    2022/3/8
* @brief CPU 周期计数、短段耗时、时间累计及等待；使用前由 BSPInit 调用 DWT_Init。
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

// 频繁调用处直接读取 CYCCNT，32 位差值允许计数从最大值回到 0 一次。
// 单次间隔须小于一个计数周期；480 MHz 时约 8.94 s，超过后无法仅凭差值知道过去多久。
extern volatile uint32_t dwt_cpu_freq_mhz; /* 由 DWT_Init() 设置 */

typedef struct
{
    volatile uint32_t last_us;  /* 最近一次耗时 */
    volatile uint32_t max_us;   /* 最大耗时 */
    volatile uint32_t total_us; /* 累计耗时(4295s 回绕, 会话内足够) */
    volatile uint32_t calls;    /* 调用次数 */
} DWT_Probe_t;

/** @brief 读取当前原始周期数，供后续计算差值。
* @return 32 位 CYCCNT 读数，不是毫秒时间。
 */
static inline uint32_t DWT_ProbeStart(void)
{
    return DWT->CYCCNT;
}

/** @brief 计算从 start 到现在经过的周期数。
* @param[in] start 先前读取的 CYCCNT，间隔须小于一次计数周期。
* @return 周期数，计数从最大值回到 0 时通过无符号相减计算。
 */
static inline uint32_t DWT_ProbeElapsedCycles(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start);
}

/** @brief 将周期差除以 CPU 的 MHz 频率，得到整数微秒。
* @param[in] start 先前读取的 CYCCNT，间隔须小于一次计数周期。
* @return 经过的 us，不足 1 us 部分不保留；频率为 0 时按 480 MHz 计算。
 */
static inline uint32_t DWT_ProbeElapsedUs(uint32_t start)
{
    uint32_t f = dwt_cpu_freq_mhz;
    if (f == 0u)
        f = 480u;
    return DWT_ProbeElapsedCycles(start) / f;
}

/** @brief 记录本次耗时，并增加累计时间和次数。
* @param[in,out] probe 有效统计对象，首次使用前清零，不检查 NULL；同一对象不要并发更新。
* @param[in] start 本段开始时的 CYCCNT；各微秒计数超过 uint32_t 范围会回到 0。
 */
static inline void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start)
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
* @param[in] CPU_Freq_mHz CPU 主频，单位 MHz；MC-02 当前传 480，须大于 0；调用会清累计时间。
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
* @param[in,out] cnt_last 先前周期读数地址，不能为 NULL；本次会改成当前读数，间隔须小于约 8.94 s（480 MHz）。
 * @return float 时间间隔,单位为秒/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
* @param[in,out] cnt_last 先前周期读数地址，不能为 NULL；本次会改成当前读数，间隔须小于约 8.94 s（480 MHz）。
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
* @param[in] Delay 等待秒数，须为正常非负数且小于一次 CYCCNT 计数周期；不挂起任务、不更新累计时间。
 */
void DWT_Delay(float Delay);

/** @brief 更新内部秒、毫秒、微秒分项；内部调用 DWT_CyclesUpdate() 累加计数。
* @note 三个 Timeline 函数也各自调用 DWT_CyclesUpdate()；累计函数至少每个计数周期内调用一次，
*       480 MHz 下间隔须小于约 8.94 s，不能靠相减识别跨过多次计数周期。
 */
void DWT_SysTimeUpdate(void);

#endif /* BSP_DWT_H_ */
