/**
 * @file bsp_init.c
 * @brief 实现 BSPInit 的启动顺序；使用条件见 bsp_init.h。
 */
#include "bsp_init.h"
#include "bsp_dwt.h"
#include "bsp_watchdog.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include "bsp_crash.h"
#include "bsp_param.h"
#include "bsp_can.h"
#include "platform_feature_startup.h"
#include "init_diagnostic.h"
#include "platform_clock_startup.h"

uint8_t BSPStartupProtectionInit(void)
{
    DWT_Init(480); // 系统时钟已经切换，建立看门狗更新等待使用的周期时基。
    return BSP_WatchdogInitChecked(4000u); // 外设阶段开始前建立启动保护。
}

uint8_t BSPInit(void)
{
    if (!BSP_WatchdogIsRunning()) return 0u; // 不重新启动看门狗，保留早期阶段的期限。
    CANInitWorkspace();
    BSPLogInit();
    BSPLogState_t log_state = {0}; // 两个出口相互独立，分别记录启动结果。
    if (BSPLogGetState(&log_state) != BSP_LOG_STATUS_OK)
        InitDiagnosticReportDegraded(INIT_STAGE_BSP, INIT_MODULE_LOG, INIT_FAILURE_RESOURCE,
                                     INIT_RESOURCE_LOG_STATUS); // 不能读取状态时不猜测哪个出口失败。
    else
    {
        if (!log_state.backend_available)
            InitDiagnosticReportDegraded(INIT_STAGE_BSP, INIT_MODULE_LOG, INIT_FAILURE_RESOURCE,
                                         INIT_RESOURCE_LOG_RTT); // RTT 失败不妨碍诊断串口。
        if (!log_state.diagnostic_uart_available)
            InitDiagnosticReportDegraded(INIT_STAGE_BSP, INIT_MODULE_LOG, INIT_FAILURE_RESOURCE,
                                         INIT_RESOURCE_LOG_DIAGNOSTIC_UART); // 串口失败不妨碍 RTT。
    }
    uint8_t adc_started = 0u; // 尚未确认 ADC 启动成功。
    if (PlatformFeatureAdcSelected() && PlatformFeatureAdcReady())
    {
        if (!PlatformPeripheralStartupEnter())
            PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE); // 期限已用尽，立即停止启动。
        BSP_ADCInit(); // 板级成功后才尝试校准并启动 DMA。
        adc_started = BSP_ADCIsStarted(); // 不用首批数据判断 DMA 是否启动。
        if (!PlatformPeripheralStartupLeave())
            PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE); // 超期后不继续初始化其他资源。
    }
    if (PlatformFeatureAdcSelected() && !adc_started)
        InitDiagnosticReportDegraded(INIT_STAGE_BSP, INIT_MODULE_ADC, INIT_FAILURE_RESOURCE, 1u); // 记录不可用的 ADC1。
    // 复位原因和上次崩溃记录需要通过已初始化的日志输出。
    BSP_WatchdogLogResetReason();
    CrashLogInit();
    // 先载入参数，供后续模块读取；沿用原流程，不在此处处理返回结果。
    ParamInit();
    return 1u; // 参数读取失败仍沿用既有默认值规则。
}
