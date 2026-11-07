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

uint8_t BSPInit(void)
{
    DWT_Init(480);
    CANInitWorkspace();
    BSPLogInit();
    // 启动先使用 4000 ms 窗口，任务创建成功后由启动协调者改为 200 ms。
    if (!BSP_WatchdogInitChecked(4000u)) return 0u; // 看门狗未确认时不能继续初始化设备。
    uint8_t adc_started = 0u; // 尚未确认 ADC 启动成功。
    if (PlatformFeatureAdcReady() && PlatformPeripheralStartupEnter())
    {
        BSP_ADCInit(); // 板级成功后才尝试校准并启动 DMA。
        const uint8_t sample_started = BSP_ADCIsStarted(); // 不用首批数据判断 DMA 是否启动。
        adc_started = (uint8_t)(PlatformPeripheralStartupLeave() && sample_started); // 同时要求操作未超期。
    }
    if (!adc_started)
        InitDiagnosticReportDegraded(INIT_STAGE_BSP, INIT_MODULE_ADC, INIT_FAILURE_RESOURCE, 1u); // 记录不可用的 ADC1。
    // 复位原因和上次崩溃记录需要通过已初始化的日志输出。
    BSP_WatchdogLogResetReason();
    CrashLogInit();
    // 先载入参数，供后续模块读取；沿用原流程，不在此处处理返回结果。
    ParamInit();
    return 1u; // 参数读取失败仍沿用既有默认值规则。
}
