#include <stdint.h>
#include <stdio.h>

#include "bsp_watchdog.h"
#include "main.h"

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);           \
            return 1;                                                           \
        }                                                                       \
    } while (0)

DWT_TypeDef mc02_test_dwt;
IWDG_TypeDef mc02_test_iwdg;
DBGMCU_TypeDef mc02_test_dbgmcu;
RCC_TypeDef mc02_test_rcc;
uint32_t mc02_test_tick;
uint32_t mc02_test_primask;
volatile uint32_t dwt_cpu_freq_mhz = 1u;

static uint32_t dwt_cycle_step;
static uint32_t error_log_count;
static uint32_t info_log_count;
static uint32_t dwt_read_count;
static uint32_t set_busy_on_read; // 指定 DWT 读取时使更新标志保持忙，模拟写后失败。
static uint8_t inject_reentry; // 配置等待中尝试喂狗和重复配置。
static uint8_t reentry_result; // 保存重入调用的结果，供测试检查。
static uint8_t feed_changed_key; // 记录等待时喂狗是否错误写入寄存器。
static uint32_t wait_irq_mask; // 记录硬件等待期间的中断状态。

uint32_t PlatformIrqSaveAndDisable(void)
{
    const uint32_t old = mc02_test_primask; // 保留调用前的中断状态。
    mc02_test_primask = 1u; // 模拟极短的本地中断屏蔽。
    return old; // 由调用方恢复原状态。
}

void PlatformIrqRestore(uint32_t previous)
{
    mc02_test_primask = previous; // 配对恢复，不能强制打开原本关闭的中断。
}

DWT_TypeDef *MC02TestDWT(void)
{
    ++dwt_read_count; // 区分写前和写后的等待。
    if (inject_reentry)
    {
        inject_reentry = 0u; // 只注入一次，避免重入本测试回调。
        wait_irq_mask = mc02_test_primask; // 等待期间应保留调用前的中断状态。
        const uint32_t key = mc02_test_iwdg.KR; // 保存当前硬件命令。
        BSP_WatchdogFeed(); // 模拟另一个任务恰在等待期间请求喂狗。
        feed_changed_key = (uint8_t)(key != mc02_test_iwdg.KR); // 未确认配置不得被重载。
        reentry_result = BSP_WatchdogInitChecked(200u); // 第二个配置调用必须被拒绝。
    }
    if (dwt_read_count == set_busy_on_read) mc02_test_iwdg.SR = IWDG_SR_PVU; // 注入写后一直未完成。
    mc02_test_dwt.CYCCNT += dwt_cycle_step;
    return &mc02_test_dwt;
}

void MC02TestLogError(void)
{
    error_log_count++;
}

void MC02TestLogInfo(void)
{
    info_log_count++;
}

int main(void)
{
    mc02_test_tick = 0u;
    mc02_test_primask = 1u;
    dwt_cycle_step = 0u;

    /* A running debugger must not freeze the watchdog counter. */
    CHECK(BSP_WatchdogInitChecked(200u)); // 正常完成寄存器更新才报告成功。
    CHECK((mc02_test_dbgmcu.APB4FZ1 & DBGMCU_APB4FZ1_DBG_IWDG1) == 0u);
    CHECK(info_log_count == 1u);
    CHECK(!BSP_WatchdogSetTimeoutChecked(UINT32_MAX)); // 非法时限不能溢出后被当作短时限。
    CHECK(BSP_WatchdogIsRunning() && BSP_WatchdogGetTimeout() == 200u); // 拒绝参数不破坏原配置。
    inject_reentry = 1u; // 在下一次实际配置等待期间注入并发调用。
    mc02_test_primask = 0u; // 模拟运行中允许中断的调用者。
    CHECK(BSP_WatchdogSetTimeoutChecked(200u));
    CHECK(!reentry_result && !feed_changed_key && !wait_irq_mask && !mc02_test_primask); // 等待不屏蔽中断。
    mc02_test_primask = 1u; // 后续继续检查调用前已屏蔽中断的情况。

    dwt_cycle_step = 10000u; // 每次读取推进一半等待时限，允许首次检查就绪。
    dwt_read_count = 0u;
    set_busy_on_read = 3u; // 第一轮检查结束后，在写后等待入口置忙。
    CHECK(!BSP_WatchdogSetTimeoutChecked(4000u)); // 写后没有确认也必须失败。
    CHECK(!BSP_WatchdogIsRunning() && BSP_WatchdogGetTimeout() == 200u);
    CHECK(mc02_test_iwdg.KR == 0x00005555u); // 只写过配置许可，没有重载未知配置。
    CHECK(error_log_count == 1u);
    set_busy_on_read = 0u;
    mc02_test_iwdg.SR = 0u;
    dwt_cycle_step = 0u;
    CHECK(BSP_WatchdogInitChecked(200u)); // 重建正常配置，再检查写前失败。

    /* A stopped HAL tick (as with interrupts masked) must not make flag polling infinite. */
    mc02_test_iwdg.SR = IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU;
    dwt_cycle_step = 10000u;
    mc02_test_iwdg.PR = 0x55u; // 保存哨兵值以检查写前超时没有覆盖寄存器。
    mc02_test_iwdg.RLR = 0x66u;
    CHECK(!BSP_WatchdogSetTimeoutChecked(4000u)); // 更新未完成时不能报告新时限生效。

    CHECK(mc02_test_tick == 0u);
    CHECK(mc02_test_primask == 1u);
    CHECK(error_log_count == 2u); // 写前失败立即退出，不重复等待。
    CHECK(mc02_test_iwdg.PR == 0x55u && mc02_test_iwdg.RLR == 0x66u);
    CHECK(!BSP_WatchdogIsRunning() && BSP_WatchdogGetTimeout() == 200u); // 不发布未确认的配置。
    mc02_test_iwdg.KR = 0u; // 检查失败后喂狗接口不会继续重载。
    BSP_WatchdogFeed();
    CHECK(mc02_test_iwdg.KR == 0u);
    CHECK(info_log_count == 2u);
    /* DWT 停止且更新标志一直不清时，检查次数上限仍必须让等待退出。 */
    dwt_cycle_step = 0u;
    CHECK(!BSP_WatchdogInitChecked(200u));
    CHECK(error_log_count == 3u && info_log_count == 2u);
    CHECK(!BSP_WatchdogIsRunning()); // 时基停滞后也不能将初始化标记为成功。
    mc02_test_iwdg.SR = 0u; // 模拟下一次启动时寄存器恢复可写。
    CHECK(BSP_WatchdogInitChecked(4000u));
    CHECK(BSP_WatchdogIsRunning() && BSP_WatchdogGetTimeout() == 4000u);
    CHECK(info_log_count == 3u);
    CHECK(BSP_WatchdogSetTimeoutChecked(32760u)); // 最大可表示时限使用 256 分频。
    CHECK(mc02_test_iwdg.PR == 6u && mc02_test_iwdg.RLR == 4095u);
    CHECK(!BSP_WatchdogSetTimeoutChecked(32761u)); // 超出硬件范围不能写入保留分频值。
    CHECK(BSP_WatchdogIsRunning() && BSP_WatchdogGetTimeout() == 32760u);
    puts("PASS: watchdog remains active in debug and flag polling is bounded without HAL tick");
    return 0;
}
