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
#include "../os/platform_irq.h"
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
#define IWDG_MAX_REQUEST_MS 32760u // 按 32 kHz、最大 256 分频和 4095 重载值计算。

// 保存请求值和本次启动的软件状态，供任务及调试器读取。
static volatile uint32_t watchdog_timeout_ms = 200;
static volatile uint8_t watchdog_started = 0;
static uint8_t watchdog_configuring; // 只允许一个调用者配置 IWDG 寄存器。
static volatile uint8_t iwdg_reset_flag = 0;

// 用短中断屏蔽保护配置状态；等待寄存器期间不保持屏蔽。
static uint8_t IWDG_BeginConfiguration(uint8_t require_started)
{
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 与喂狗检查使用同一个保护。
    if (watchdog_configuring || (require_started && !watchdog_started))
    {
        PlatformIrqRestore(previous); // 已有配置或尚未启动时不改变原状态。
        return 0u; // 拒绝重入，调用者不触碰硬件。
    }
    watchdog_configuring = 1u; // 为本次配置保留唯一写入权。
    watchdog_started = 0u; // 先禁止其他任务在寄存器更新期间喂狗。
    PlatformIrqRestore(previous); // 之后允许中断，等待不得阻塞中断。
    return 1u; // 调用者取得配置权。
}

// 发布完成或失败，只有确认成功时才重新允许喂狗。
static void IWDG_EndConfiguration(uint32_t timeout_ms, uint8_t success)
{
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 防止喂狗观察到部分更新。
    if (success) watchdog_timeout_ms = timeout_ms; // 只发布硬件已接受的请求时限。
    watchdog_started = success; // 失败时保持禁止喂狗。
    watchdog_configuring = 0u; // 结束本次配置并允许后续显式重试。
    PlatformIrqRestore(previous); // 恢复进入前的中断状态。
}

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
static uint8_t IWDG_WritePrescalerAndReload(uint32_t timeout_ms)
{
    uint32_t reload = 0;
    uint32_t prescaler = 4;
    uint32_t pr = 0;

    // 换算时将 0 改为 1ms，外面的软件请求记录不随这里改变。
    if (timeout_ms == 0)
        timeout_ms = 1;

    // 从小到大尝试受支持的 4～256 分频，重载值落在 1～4095 时停止。
    for (pr = 0; pr <= 6; ++pr)
    {
        prescaler = 4u << pr;
        reload = (timeout_ms * (IWDG_LSI_HZ / 1000u)) / prescaler;
        if (reload <= 0xFFFu && reload > 0)
            break;
    }

    // 防御性检查：调用前已限制请求，不能写入不支持的分频档位。
    if (reload > 0xFFFu)
        return 0u;
    // 计算结果为 0 时写入 1，避免将重载值设为 0。
    if (reload == 0)
        reload = 1;

    // 写前等上一次更新完成；超时后不能写入尚未就绪的寄存器。
    if (!IWDG_WaitForUpdateFlags())
    {
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_01);
        return 0u; // 退出后由调用方禁止继续喂狗。
    }

    IWDG1->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
    IWDG1->PR = pr;
    IWDG1->RLR = reload;

    // 写后再次等待，确保新时限已经被硬件接受。
    if (!IWDG_WaitForUpdateFlags())
    {
        BSP_LOG_EVENT_0(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_02);
        return 0u; // 不重载未经确认的时限。
    }

    IWDG1->KR = IWDG_KEY_RELOAD;
    return 1u; // 两次等待均完成后才报告成功。
}

void BSP_WatchdogInit(uint32_t timeout_ms)
{
    (void)BSP_WatchdogInitChecked(timeout_ms); // 保留旧入口，关键调用应检查新接口的结果。
}

uint8_t BSP_WatchdogInitChecked(uint32_t timeout_ms)
{
    if (timeout_ms > IWDG_MAX_REQUEST_MS) return 0u; // 超出可表示范围时不触碰现有配置。
    if (!IWDG_BeginConfiguration(0u)) return 0u; // 重入时不影响正在进行的配置。
    // 先发启动命令，让看门狗时钟开始工作，再配置分频和重载值。
    IWDG1->KR = IWDG_KEY_ENABLE;
    const uint8_t configured = IWDG_WritePrescalerAndReload(timeout_ms); // 在中断可运行时等待硬件。
    IWDG_EndConfiguration(timeout_ms, configured); // 统一发布结果和释放配置权。
    if (!configured) return 0u; // 等待失败时保持禁止喂狗状态。

    BSP_LOG_EVENT_1(EV_BSP_WATCHDOG_BSP_WATCHDOG_C_03, BSP_LOG_FIELD_U32((unsigned)timeout_ms));
    return 1u; // 成功状态只在完整配置后发布。
}

void BSP_WatchdogFeed(void)
{
    // 本次启动尚未完成初始化时，不写喂狗寄存器。
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 与禁止喂狗的状态切换保持原子。
    if (watchdog_started)
        IWDG1->KR = IWDG_KEY_RELOAD;
    PlatformIrqRestore(previous); // 恢复原有中断状态，不跨越硬件等待。
}

void BSP_WatchdogSetTimeout(uint32_t timeout_ms)
{
    (void)BSP_WatchdogSetTimeoutChecked(timeout_ms); // 保留旧入口，维护调用应检查结果。
}

uint8_t BSP_WatchdogSetTimeoutChecked(uint32_t timeout_ms)
{
    if (timeout_ms > IWDG_MAX_REQUEST_MS) return 0u; // 先拒绝超范围参数，保留当前配置。
    if (!IWDG_BeginConfiguration(1u)) return 0u; // 未启动或已有配置时不能修改寄存器。
    const uint8_t configured = IWDG_WritePrescalerAndReload(timeout_ms); // 等待期间不屏蔽中断。
    IWDG_EndConfiguration(timeout_ms, configured); // 成功才发布并允许喂狗。
    return configured; // 向调用方报告硬件更新是否确认。
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
