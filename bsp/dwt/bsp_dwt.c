/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author modified by Neo with annotation
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 */

#include "bsp_dwt.h"
#include "cmsis_os.h"

static DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static float CPU_FREQ_Hz_inv;    /* 1/CPU_FREQ_Hz   : 时间轴用, 省掉每次除法 */
static float CPU_FREQ_Hz_ms_inv; /* 1000/CPU_FREQ_Hz */

/* ---- 时间轴: 增量累加(2026-09 重写) ----
   原实现是 "CYCCNT_RountCount * UINT32_MAX + cnt_now", 三个问题:
     1) 模数应该写 2^32, 写 UINT32_MAX 会让每次回绕少算 1 个计数;
     2) 每次调用都要做 64bit 除法(在 1kHz 热路径上很贵);
     3) 最要命的是 -O2 下编译器按寄存器变量/别名规则把
        "读 CYCCNT → 比较 → 累加 → 写回" 这段序列重排掉, 实测时间轴返回值
        根本不前进 → EKF 的 dt 恒为 0、标定的 12s 超时永不触发(启动卡死)。
   现在改成: 每次只读一次 CYCCNT, 用 32bit 无符号减法得到"距上次的周期数"
   (回绕天然正确: (a-b) 在模 2^32 下永远等于真实增量), 再累加进 64bit 总数。
   热路径只有一次减法 + 一次 64bit 加法, 没有除法、没有浮点。 */
static volatile uint64_t s_cycles_total = 0; /* 上电以来的 CPU 周期总数 */
static volatile uint32_t s_cycles_last = 0;  /* 上次采样到的 CYCCNT */

/* 热路径探针的除数(见 bsp_dwt.h)。给初值, 防止 DWT_Init 之前被调用时除 0 */
volatile uint32_t dwt_cpu_freq_mhz = 480u;

/**
 * @brief 把"从上一次调用到现在的周期数"累加进 64bit 总计数
 * @note  读-改-写必须原子。若两个任务/中断交错执行, 会出现
 *        "同一段增量被加两次" 或 "s_cycles_last 倒退导致时间跳变",
 *        所以用临界区包住(PRIMASK 保护, 同时挡住任务切换和中断)。
 */
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

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz)
        ;
}
