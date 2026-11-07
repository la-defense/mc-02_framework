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
#include <string.h>

uint8_t BSPInitCallerA(void);
uint8_t BSPInitCallerB(void);

static unsigned step;
static uint8_t watchdog_failure; // 按场景模拟早期看门狗初始化失败。
static uint8_t adc_board_failure, adc_start_failure;
static unsigned degraded_count;

uint8_t PlatformFeatureAdcReady(void) { return (uint8_t)!adc_board_failure; }
uint8_t BSP_ADCIsStarted(void) { return (uint8_t)!adc_start_failure; }
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
    adc_board_failure = (uint8_t)(strcmp(argv[1], "adc-board-fail") == 0);
    adc_start_failure = (uint8_t)(strcmp(argv[1], "adc-start-fail") == 0);
    if (strcmp(argv[1], "a") == 0 || watchdog_failure || adc_board_failure || adc_start_failure)
        TEST_CHECK(BSPInitCallerA() == (uint8_t)!watchdog_failure);
    else
    {
        TEST_CHECK(strcmp(argv[1], "b") == 0);
        TEST_CHECK(BSPInitCallerB());
    }
    TEST_CHECK(step == (watchdog_failure ? 4u : 8u)); // 失败后禁止继续初始化 ADC 和参数。
    TEST_CHECK(degraded_count == (unsigned)(adc_board_failure || adc_start_failure)); // 两类 ADC 失败都只报告一次。
    return 0;
}
