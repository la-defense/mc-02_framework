/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief 读取 CPU 周期差并累计时间，提供任务和中断可用的短时间测量。
 */

#include "bsp_dwt.h"
#include "cmsis_os.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static float CPU_FREQ_Hz_inv;    /* 1/CPU_FREQ_Hz   : 时间轴用, 省掉每次除法 */
static float CPU_FREQ_Hz_ms_inv; /* 1000/CPU_FREQ_Hz */

// 每次取当前周期数与上次读数之差，再加到 64 位总量；两次读取须间隔小于一次计数周期。
static volatile uint64_t s_cycles_total = 0; /* 上电以来的 CPU 周期总数 */
static volatile uint32_t s_cycles_last = 0;  /* 上次采样到的 CYCCNT */

/* 热路径探针的除数(见 bsp_dwt.h)。给初值, 防止 DWT_Init 之前被调用时除 0 */
volatile uint32_t dwt_cpu_freq_mhz = 480u;

// 暂时关闭普通中断完成读、加、写，避免不同调用把同一段周期重复累加，随后恢复原状态。
static void DWT_CyclesUpdate(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t cnt_now = DWT->CYCCNT;
    s_cycles_total += (uint64_t)(uint32_t)(cnt_now - s_cycles_last);
    s_cycles_last = cnt_now;

    __set_PRIMASK(primask);
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CPU_FREQ_Hz_inv = 1.0f / (float)CPU_FREQ_Hz;
    CPU_FREQ_Hz_ms_inv = 1000.0f / (float)CPU_FREQ_Hz;
    dwt_cpu_freq_mhz = CPU_Freq_mHz;

    /* 时间轴清零, 并以当前 CYCCNT 作为增量累加的起点 */
    s_cycles_total = 0;
    s_cycles_last = DWT->CYCCNT;
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    /* 32bit 无符号减法: 回绕时 (cnt_now - *cnt_last) 依然等于真实增量 */
    float dt = ((uint32_t)(cnt_now - *cnt_last)) * CPU_FREQ_Hz_inv;
    *cnt_last = cnt_now;

    DWT_CyclesUpdate();

    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = (double)((uint32_t)(cnt_now - *cnt_last)) / (double)(CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    DWT_CyclesUpdate();

    return dt;
}

void DWT_SysTimeUpdate(void)
{
    DWT_CyclesUpdate();

    uint64_t cycles = s_cycles_total;
    uint32_t rem = (uint32_t)(cycles % CPU_FREQ_Hz); /* 不足 1s 的部分 */

    SysTime.s = (uint32_t)(cycles / CPU_FREQ_Hz);
    SysTime.ms = (uint16_t)(rem / CPU_FREQ_Hz_ms);
    SysTime.us = (uint16_t)((rem % CPU_FREQ_Hz_ms) / CPU_FREQ_Hz_us);
}

float DWT_GetTimeline_s(void)
{
    DWT_CyclesUpdate();
    return (float)s_cycles_total * CPU_FREQ_Hz_inv;
}

float DWT_GetTimeline_ms(void)
{
    DWT_CyclesUpdate();
    return (float)s_cycles_total * CPU_FREQ_Hz_ms_inv;
}

uint64_t DWT_GetTimeline_us(void)
{
    DWT_CyclesUpdate();
    return s_cycles_total / (uint64_t)CPU_FREQ_Hz_us;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = Delay;

    // 只轮询 CPU 周期数，不依赖 HAL 毫秒中断，也不让出当前任务。
    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
