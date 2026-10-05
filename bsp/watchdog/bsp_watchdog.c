#include "bsp_watchdog.h"
#include "main.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

#define IWDG_KEY_RELOAD 0x0000AAAAu
#define IWDG_KEY_ENABLE 0x0000CCCCu
#define IWDG_KEY_WRITE_ACCESS_ENABLE 0x00005555u

/* LSI 典型值 32kHz, 这里按 32kHz 计算分频和重载值 */
#define IWDG_LSI_HZ 32000u
#define IWDG_UPDATE_FLAGS (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)
#define IWDG_UPDATE_TIMEOUT_MS 20u

/* volatile: 这些状态既会被任务读写, 也可能在调试/故障注入时被调试器修改,
   不能假设编译器可以缓存 */
static volatile uint32_t watchdog_timeout_ms = 200;
static volatile uint8_t watchdog_started = 0;
static volatile uint8_t iwdg_reset_flag = 0;

static uint8_t IWDG_WaitForUpdateFlags(void)
{
    uint32_t cycles_per_ms = dwt_cpu_freq_mhz * 1000u;
    if (cycles_per_ms == 0u)
        cycles_per_ms = 480000u;
    const uint32_t timeout_cycles = cycles_per_ms * IWDG_UPDATE_TIMEOUT_MS;
    const uint32_t start = DWT->CYCCNT;

    while (IWDG1->SR & IWDG_UPDATE_FLAGS)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles)
            return 0;
    }
    return 1;
}

static void IWDG_WritePrescalerAndReload(uint32_t timeout_ms)
{
    uint32_t reload = 0;
    uint32_t prescaler = 4;
    uint32_t pr = 0;

    if (timeout_ms == 0)
        timeout_ms = 1;

    /* 选择最小的分频系数, 使 reload <= 0xFFF */
    for (pr = 0; pr <= 7; ++pr)
    {
        prescaler = 4u << pr;
        reload = (timeout_ms * (IWDG_LSI_HZ / 1000u)) / prescaler;
        if (reload <= 0xFFFu && reload > 0)
            break;
    }

    if (reload > 0xFFFu)
    {
        reload = 0xFFFu;
        pr = 7;
    }
    if (reload == 0)
        reload = 1;

    /* 等待上一次分频/重载更新完成后再写新值; 超时则继续, 避免死等 */
    if (!IWDG_WaitForUpdateFlags())
    {
        LOGERROR("[watchdog] IWDG update flags stuck before write");
    }

    IWDG1->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
    IWDG1->PR = pr;
    IWDG1->RLR = reload;

    if (!IWDG_WaitForUpdateFlags())
    {
        LOGERROR("[watchdog] IWDG update flags stuck after write");
    }

    IWDG1->KR = IWDG_KEY_RELOAD;
}

void BSP_WatchdogInit(uint32_t timeout_ms)
{
    watchdog_timeout_ms = timeout_ms;

    /* 必须先启动 IWDG 让 LSI 起振, 否则写 PR/RLR 后 PVU/RVU 永远不会清零.
       顺序与 HAL_IWDG_Init 保持一致. */
    IWDG1->KR = IWDG_KEY_ENABLE;
    IWDG_WritePrescalerAndReload(timeout_ms);
    watchdog_started = 1;
    BSP_WatchdogFeed();

    LOGINFO("[watchdog] IWDG started, timeout=%ums", (unsigned)timeout_ms);
}

void BSP_WatchdogFeed(void)
{
    if (watchdog_started)
        IWDG1->KR = IWDG_KEY_RELOAD;
}

void BSP_WatchdogSetTimeout(uint32_t timeout_ms)
{
    watchdog_timeout_ms = timeout_ms;
    if (watchdog_started)
        IWDG_WritePrescalerAndReload(timeout_ms);
}

uint32_t BSP_WatchdogGetTimeout(void)
{
    return watchdog_timeout_ms;
}

void BSP_WatchdogLogResetReason(void)
{
    uint32_t rsr = RCC->RSR;

    if (rsr & RCC_RSR_IWDG1RSTF)
    {
        iwdg_reset_flag = 1;
        LOGERROR("[watchdog] reset reason: IWDG1");
    }
#ifdef RCC_RSR_WWD1RSTF
    if (rsr & RCC_RSR_WWD1RSTF)
        LOGERROR("[watchdog] reset reason: WWDG1");
#endif
    if (rsr & RCC_RSR_PORRSTF)
        LOGINFO("[watchdog] reset reason: POR/PDR");
    if (rsr & RCC_RSR_PINRSTF)
        LOGINFO("[watchdog] reset reason: NRST pin");
    if (rsr & RCC_RSR_BORRSTF)
        LOGERROR("[watchdog] reset reason: BOR");
#ifdef RCC_RSR_SFTRSTF
    if (rsr & RCC_RSR_SFTRSTF)
        LOGINFO("[watchdog] reset reason: software");
#endif
#ifdef RCC_RSR_LPWRRSTF
    if (rsr & RCC_RSR_LPWRRSTF)
        LOGERROR("[watchdog] reset reason: low power");
#endif

    /* 清除复位标志, 供下一次启动判断 */
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
