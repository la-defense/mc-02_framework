/**
 * @file bsp_init_link_test.c
 * @brief 主机验证两个源文件的链接及启动顺序；每个进程只调用一次 BSPInit。
 * @note 参数 a/b 选择调用方，底层只记录调用，不操作硬件。
 */
#include "test_check.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_watchdog.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include "bsp_crash.h"
#include "bsp_param.h"
#include "init_diagnostic.h"
#include "platform_clock_startup.h"
#include <setjmp.h>
#include <string.h>

uint8_t BSPInitCallerA(void);
uint8_t BSPInitCallerB(void);

static unsigned step;
static uint8_t watchdog_failure; // 按场景模拟早期看门狗初始化失败。
static uint8_t adc_board_failure, adc_start_failure;
static uint8_t adc_disabled;
static uint8_t deadline_enter_failure, deadline_leave_failure;
static unsigned degraded_count;
static jmp_buf deadline_exit;

uint8_t PlatformFeatureAdcReady(void) { return (uint8_t)!adc_board_failure; }
uint8_t PlatformFeatureAdcSelected(void) { return (uint8_t)!adc_disabled; }
uint8_t BSP_ADCIsStarted(void) { return (uint8_t)!adc_start_failure; }
uint8_t PlatformPeripheralStartupEnter(void) { return (uint8_t)!deadline_enter_failure; }
uint8_t PlatformPeripheralStartupLeave(void) { return (uint8_t)!deadline_leave_failure; }
_Noreturn void PlatformClockStartupFail(PlatformClockFailure_t failure)
{
    TEST_CHECK(failure == PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE); // 保留外设阶段的具体原因。
    longjmp(deadline_exit, 1); // 模拟关断及复位，不执行后续初始化。
}
void InitDiagnosticReportDegraded(int stage, uint8_t module, int reason, uint16_t resource_id)
{
    TEST_CHECK(stage == INIT_STAGE_BSP && module == INIT_MODULE_ADC && reason == INIT_FAILURE_RESOURCE && resource_id == 1u);
    ++degraded_count; // 记录辅助 ADC 失败，不终止后续启动。
    if (adc_board_failure) ++step; // 未调用 ADC 时保留测试阶段位置。
}

void DWT_Init(uint32_t cpu_mhz)
{
    TEST_CHECK(step == 0u);
    TEST_CHECK(cpu_mhz == 480u);
    ++step;
}

uint8_t BSP_WatchdogInitChecked(uint32_t timeout_ms)
{
    TEST_CHECK(step == 3u);
    TEST_CHECK(timeout_ms == 4000u);
    ++step;
    return (uint8_t)!watchdog_failure; // 失败时下游资源不应再启动。
}

void BSPLogInit(void)
{
    TEST_CHECK(step == 2u);
    ++step;
}

void CANInitWorkspace(void)
{
    TEST_CHECK(step == 1u);
    ++step;
}

void BSP_ADCInit(void)
{
    TEST_CHECK(step == 4u);
    ++step;
}

void BSP_WatchdogLogResetReason(void)
{
    if (adc_disabled) ++step; // 未选择 ADC 时，测试跳过其启动位置。
    TEST_CHECK(step == 5u);
    ++step;
}

void CrashLogInit(void)
{
    TEST_CHECK(step == 6u);
    ++step;
}

uint8_t ParamInit(void)
{
    TEST_CHECK(step == 7u);
    ++step;
    // 即使参数载入失败，原有 void 启动入口也不汇总失败，不能据此宣称整体成功。
    return 0u;
}

/** @brief 选择一个调用方；任何顺序、参数或次数不符均以非零退出。 */
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    watchdog_failure = (uint8_t)(strcmp(argv[1], "watchdog-fail") == 0); // 单独检查失败路径。
    adc_disabled = (uint8_t)(strcmp(argv[1], "adc-disabled") == 0);
    adc_board_failure = (uint8_t)(strcmp(argv[1], "adc-board-fail") == 0);
    adc_start_failure = (uint8_t)(strcmp(argv[1], "adc-start-fail") == 0);
    deadline_enter_failure = (uint8_t)(strcmp(argv[1], "deadline-enter-fail") == 0);
    deadline_leave_failure = (uint8_t)(strcmp(argv[1], "deadline-leave-fail") == 0);
    if (deadline_enter_failure || deadline_leave_failure)
    {
        if (setjmp(deadline_exit) == 0) (void)BSPInitCallerA(); // 超期必须进入关断复位入口。
        TEST_CHECK(step == (deadline_enter_failure ? 4u : 5u)); // 不继续记录复位原因或读取参数。
        TEST_CHECK(degraded_count == 0u); // 外设总期限失败属于核心故障。
        return 0;
    }
    if (strcmp(argv[1], "a") == 0 || watchdog_failure || adc_disabled || adc_board_failure || adc_start_failure ||
        deadline_enter_failure || deadline_leave_failure)
        TEST_CHECK(BSPInitCallerA() == (uint8_t)!watchdog_failure);
    else
    {
        TEST_CHECK(strcmp(argv[1], "b") == 0);
        TEST_CHECK(BSPInitCallerB());
    }
    TEST_CHECK(step == (watchdog_failure ? 4u : 8u)); // 失败后禁止继续初始化 ADC 和参数。
    TEST_CHECK(degraded_count == (unsigned)(adc_board_failure || adc_start_failure)); // 辅助 ADC 失败只报告一次。
    return 0;
}
