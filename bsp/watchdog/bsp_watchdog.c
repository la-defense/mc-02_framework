/**
 * @file bsp_watchdog.c
 * @brief 实现 IWDG1 的时限换算、喂狗和复位来源记录，接口用法见 bsp_watchdog.h。
 * 配置寄存器时用 DWT 计数等待更新；HAL tick 停止时，只要 DWT 仍计数就能检查等待时间。
 */
#include "bsp_watchdog.h"
#include "main.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "../os/platform_startup_wait.h"
#include <limits.h>
#include <stddef.h>

// 写入 KR 的三条命令分别用于重载计数、启动看门狗和允许修改配置。
#define IWDG_KEY_RELOAD 0x0000AAAAu
#define IWDG_KEY_ENABLE 0x0000CCCCu
#define IWDG_KEY_WRITE_ACCESS_ENABLE 0x00005555u

// 按 LSI 为 32kHz 估算分频和重载值，实际时限会受时钟频率影响。
#define IWDG_LSI_HZ 32000u
#define IWDG_UPDATE_FLAGS (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)
#define IWDG_UPDATE_TIMEOUT_MS 20u

// 保存请求值和本次启动的软件状态，供任务及调试器读取。
static volatile uint32_t watchdog_timeout_ms = 200;
static volatile uint8_t watchdog_started = 0;
static volatile uint8_t iwdg_reset_flag = 0;

// 读取看门狗更新标记；清零后才允许下一次寄存器配置。
static uint8_t IWDG_UpdateReady(void *context)
{
    (void)context; // 硬件寄存器不需要调用方上下文。
    return (uint8_t)((IWDG1->SR & IWDG_UPDATE_FLAGS) == 0u); // 全部更新完成才返回成功。
}

// 获取当前 DWT 计数，供等待函数计算固定期限和回绕。
static uint32_t IWDG_ReadCycles(void *context)
{
    (void)context; // 时钟来源由本板固定选择。
    return DWT->CYCCNT; // 不依赖由中断更新的 HAL tick。
}

// DWT 正常时按周期限制等待；计数停止时由检查次数上限退出。
static uint8_t IWDG_WaitForUpdateFlags(void)
{
    uint32_t frequency_mhz = dwt_cpu_freq_mhz; // 固定读取一次频率，避免等待期间改变总期限。
    // 没有频率记录时按当前板的 480MHz 估算，仍须保证计数器已启动。
    if (frequency_mhz == 0u)
        frequency_mhz = 480u; // 没有频率记录时保留原有 H723 默认值。
    const uint64_t cycles = (uint64_t)frequency_mhz * 1000u * IWDG_UPDATE_TIMEOUT_MS; // 使用宽整数避免乘法溢出。
    if (cycles == 0u || cycles > INT32_MAX) return 0u; // 回绕判断要求期限在半个周期范围内。
    const uint32_t timeout_cycles = (uint32_t)cycles; // 范围检查后才转换为 DWT 位宽。
    const uint32_t max_polls = timeout_cycles + 1u; // 一次检查至少耗费一个 CPU 周期；DWT 停止也可退出。
    return (uint8_t)(PlatformStartupWait(IWDG_UpdateReady, IWDG_ReadCycles, NULL,
                                          timeout_cycles, max_polls) == STARTUP_WAIT_READY);
}

// 将请求时限换算后写入 PR/RLR，并重载计数；写前、写后各自等待更新。
static void IWDG_WritePrescalerAndReload(uint32_t timeout_ms)
{
    uint32_t reload = 0;
    uint32_t prescaler = 4;
    uint32_t pr = 0;

    // 换算时将 0 改为 1ms，外面的软件请求记录不随这里改变。
    if (timeout_ms == 0)
        timeout_ms = 1;

    // 从小到大尝试计算所用的分频值，重载值落在 1～4095 时停止。
    for (pr = 0; pr <= 7; ++pr)
    {
        prescaler = 4u << pr;
        reload = (timeout_ms * (IWDG_LSI_HZ / 1000u)) / prescaler;
        if (reload <= 0xFFFu && reload > 0)
            break;
    }

    // 计算结果仍太大时，使用重载上限 4095 和 pr=7 继续配置。
    if (reload > 0xFFFu)
    {
        reload = 0xFFFu;
        pr = 7;
    }
    // 计算结果为 0 时写入 1，避免将重载值设为 0。
    if (reload == 0)
        reload = 1;

    // 写前等上一次更新完成；超时只输出错误，仍会执行下面的寄存器写入。
    if (!IWDG_WaitForUpdateFlags())
    {
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_01);
    }

    IWDG1->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
    IWDG1->PR = pr;
    IWDG1->RLR = reload;

    // 写后再等一次；即使超时，后面仍会执行重载命令。
    if (!IWDG_WaitForUpdateFlags())
    {
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_02);
    }

    IWDG1->KR = IWDG_KEY_RELOAD;
}

void BSP_WatchdogInit(uint32_t timeout_ms)
{
    watchdog_timeout_ms = timeout_ms;

    // 先发启动命令，让看门狗时钟开始工作，再配置分频和重载值。
    IWDG1->KR = IWDG_KEY_ENABLE;
    IWDG_WritePrescalerAndReload(timeout_ms);
    // 记录本接口已完成初始化；上面的等待超时不会阻止设置这个软件标志。
    watchdog_started = 1;
    BSP_WatchdogFeed();

    BSP_LOG_EVENT_1(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_03, BSP_LOG_FIELD_U32((unsigned)timeout_ms));
}

void BSP_WatchdogFeed(void)
{
    // 本次启动尚未完成初始化时，不写喂狗寄存器。
    if (watchdog_started)
        IWDG1->KR = IWDG_KEY_RELOAD;
}

void BSP_WatchdogSetTimeout(uint32_t timeout_ms)
{
    watchdog_timeout_ms = timeout_ms;
    // 已初始化才改硬件；未初始化时只保留上面的请求值。
    if (watchdog_started)
        IWDG_WritePrescalerAndReload(timeout_ms);
}

uint32_t BSP_WatchdogGetTimeout(void)
{
    return watchdog_timeout_ms;
}

void BSP_WatchdogLogResetReason(void)
{
    // 先读出所有标志再逐项检查，同一次启动可能记录多个来源。
    uint32_t rsr = RCC->RSR;

    // 发现过 IWDG1 复位后，软件保留这份信息，直到下次启动重新清零。
    if (rsr & RCC_RSR_IWDG1RSTF)
    {
        iwdg_reset_flag = 1;
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_04);
    }
#ifdef RCC_RSR_WWD1RSTF
    if (rsr & RCC_RSR_WWD1RSTF) // 窗口看门狗
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_05);
#endif
    if (rsr & RCC_RSR_PORRSTF) // 上电或掉电相关复位
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_06);
    if (rsr & RCC_RSR_PINRSTF) // 外部 NRST 引脚
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_07);
    if (rsr & RCC_RSR_BORRSTF) // 欠压复位
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_08);
#ifdef RCC_RSR_SFTRSTF
    if (rsr & RCC_RSR_SFTRSTF) // 软件复位
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_09);
#endif
#ifdef RCC_RSR_LPWRRSTF
    if (rsr & RCC_RSR_LPWRRSTF) // 低功耗相关复位
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_10);
#endif

    // 最后清除硬件标志，避免下次启动重复报告；已保存的软件信息继续保留。
    RCC->RSR |= RCC_RSR_RMVF;
}

uint8_t BSP_WatchdogWasIwdgReset(void)
{
    return iwdg_reset_flag;
}

uint8_t BSP_WatchdogIsRunning(void)
{
    return watchdog_started;
}
