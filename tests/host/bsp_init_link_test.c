/**
 * @file bsp_init_link_test.c
 * @brief 主机验证两个源文件的链接及启动顺序；每个进程只调用一次 BSPInit。
 * @note 参数 a/b 选择调用方，底层只记录调用，不操作硬件。
 */
#include "test_check.h"
#include "bsp_dwt.h"
#include "bsp_init.h"
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
static uint8_t watchdog_running; // 只有早期配置成功后才允许继续 BSP 初始化。
static uint8_t adc_board_failure, adc_start_failure;
static uint8_t adc_disabled;
static uint8_t deadline_enter_failure, deadline_leave_failure;
static unsigned degraded_count;
static uint8_t log_rtt_failure, log_uart_failure, log_query_failure;
static unsigned log_degraded_count;
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
    if (module == INIT_MODULE_LOG)
    {
        TEST_CHECK(stage == INIT_STAGE_BSP && reason == INIT_FAILURE_RESOURCE);
        TEST_CHECK((resource_id == INIT_RESOURCE_LOG_STATUS && log_query_failure) ||
                   (resource_id == INIT_RESOURCE_LOG_RTT && log_rtt_failure && !log_query_failure) ||
                   (resource_id == INIT_RESOURCE_LOG_DIAGNOSTIC_UART && log_uart_failure && !log_query_failure));
        ++log_degraded_count; // 日志出口降级不终止 ADC 或参数初始化。
        return;
    }
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
    TEST_CHECK(step == 1u);
    TEST_CHECK(timeout_ms == 4000u);
    ++step;
    watchdog_running = (uint8_t)!watchdog_failure; // 模拟硬件配置结果。
    return watchdog_running;
}

uint8_t BSP_WatchdogIsRunning(void) { return watchdog_running; }

void BSPLogInit(void)
{
    TEST_CHECK(step == 3u);
    ++step;
}

BSPLogStatus_t BSPLogGetState(BSPLogState_t *state)
{
    TEST_CHECK(step == 4u && state != NULL); // 日志初始化后、ADC 启动前查询。
    if (log_query_failure) return BSP_LOG_STATUS_NOT_INITIALIZED; // 不能推测无法读取的状态。
    state->backend_available = (uint8_t)!log_rtt_failure; // RTT 独立于串口。
    state->diagnostic_uart_available = (uint8_t)!log_uart_failure; // 模拟诊断串口注册结果。
    return BSP_LOG_STATUS_OK;
}

void CANInitWorkspace(void)
{
    TEST_CHECK(step == 2u);
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
    log_rtt_failure = (uint8_t)(strcmp(argv[1], "log-rtt-fail") == 0 || strcmp(argv[1], "log-both-fail") == 0);
    log_uart_failure = (uint8_t)(strcmp(argv[1], "log-uart-fail") == 0 || strcmp(argv[1], "log-both-fail") == 0);
    log_query_failure = (uint8_t)(strcmp(argv[1], "log-query-fail") == 0);
    adc_disabled = (uint8_t)(strcmp(argv[1], "adc-disabled") == 0);
    adc_board_failure = (uint8_t)(strcmp(argv[1], "adc-board-fail") == 0);
    adc_start_failure = (uint8_t)(strcmp(argv[1], "adc-start-fail") == 0);
    deadline_enter_failure = (uint8_t)(strcmp(argv[1], "deadline-enter-fail") == 0);
    deadline_leave_failure = (uint8_t)(strcmp(argv[1], "deadline-leave-fail") == 0);
    TEST_CHECK(BSPStartupProtectionInit() == (uint8_t)!watchdog_failure); // 模拟外设之前的早期保护。
    if (watchdog_failure)
    {
        TEST_CHECK(!BSPInitCallerA()); // 保护失败后不得执行其他 BSP 步骤。
        TEST_CHECK(step == 2u && degraded_count == 0u);
        return 0;
    }
    if (deadline_enter_failure || deadline_leave_failure)
    {
        if (setjmp(deadline_exit) == 0) (void)BSPInitCallerA(); // 超期必须进入关断复位入口。
        TEST_CHECK(step == (deadline_enter_failure ? 4u : 5u)); // 不继续记录复位原因或读取参数。
        TEST_CHECK(degraded_count == 0u); // 外设总期限失败属于核心故障。
        return 0;
    }
    if (strcmp(argv[1], "a") == 0 || watchdog_failure || adc_disabled || adc_board_failure || adc_start_failure ||
        log_rtt_failure || log_uart_failure || log_query_failure ||
        deadline_enter_failure || deadline_leave_failure)
        TEST_CHECK(BSPInitCallerA() == (uint8_t)!watchdog_failure);
    else
    {
        TEST_CHECK(strcmp(argv[1], "b") == 0);
        TEST_CHECK(BSPInitCallerB());
    }
    TEST_CHECK(step == 8u); // 所有已选步骤恰好执行一次。
    TEST_CHECK(degraded_count == (unsigned)(adc_board_failure || adc_start_failure)); // 辅助 ADC 失败只报告一次。
    TEST_CHECK(log_degraded_count == (unsigned)(log_rtt_failure + log_uart_failure + log_query_failure));
    return 0;
}
