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

void BSPInit(void)
{
    DWT_Init(480);
    CANInitWorkspace();
    BSPLogInit();
    // 启动先等待 4000 ms，任务创建成功后由 RobotInit 改为 200 ms。
    BSP_WatchdogInit(4000);
    BSP_ADCInit();
    // 复位原因和上次崩溃记录需要通过已初始化的日志输出。
    BSP_WatchdogLogResetReason();
    CrashLogInit();
    // 先载入参数，供后续模块读取；沿用原流程，不在此处处理返回结果。
    ParamInit();
}
