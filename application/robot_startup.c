/** @file robot_startup.c
 * @brief 在调度器运行后完成姿态初始化、任务创建和统一放行。
 */
#include "robot_startup.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "ins_startup.h"
#include "robot.h"
#include "robot_task_create.h"
#include "platform_os.h"
#include "task_monitor.h"
#include "init_diagnostic.h"
#include "bsp_watchdog.h"
#include "bsp_safety.h"
#include "robot_safety.h"

// 启动只允许唯一协调任务执行一次，失败后必须重启。
static uint8_t startup_started;

// 保留首次失败原因，并立即撤销许可与关闭板级输出。
static void RobotStartupLatchFailure(InitDiagnosticStage_e stage, uint8_t module,
                                     InitDiagnosticReason_e reason, uint16_t resource)
{
    if (!InitDiagnosticIsFailed()) // 下层已记录具体原因时不覆盖它。
        InitDiagnosticReportFailure(stage, module, reason, resource); // 保存协调失败的位置。
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE); // 发布整车初始化故障。
    RobotSafetyRequestDisable(); // 撤销业务运行许可。
    BSP_SafetyStartupLatchOutputsOff(); // 立即关闭受控输出。
}

uint8_t RobotStartupRun(void)
{
    if (startup_started) return 0u; // 不允许重复运行或重试失败的启动。
    startup_started = 1u; // 在任何可能等待的步骤之前登记本次调用。
    if (!PlatformSchedulerIsRunning()) // 设备初始化需要调度器和中断。
    {
        RobotStartupLatchFailure(INIT_STAGE_FREERTOS, INIT_MODULE_FREERTOS,
                                 INIT_FAILURE_DEPENDENCY, 0u); // 保留错误阶段。
        return 0u; // 没有调度器时不访问设备。
    }
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION); // 传感器等待期间显示当前设备初始化阶段。
    if (!RobotInitializeApplications()) // 在允许中断和任务等待的环境初始化业务模块。
    {
        RobotStartupLatchFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                 INIT_FAILURE_DEPENDENCY, 0u); // 保留下层首先记录的原因。
        return 0u; // 核心业务失败时不初始化姿态或创建周期任务。
    }
#if MC02_FEATURE_IMU
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_IMU)) // 只初始化启动计划接受的姿态模块。
#endif
    {
        const RobotInsStartupResult_e ins = RobotInitializeIns(); // 在任务上下文内采样和标定。
        if (ins == ROBOT_INS_STARTUP_CORE_FAILURE) // 辅助失败由初始化步骤记录并移除模块。
        {
            RobotStartupLatchFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                     INIT_FAILURE_RESOURCE, 0u); // 保持核心故障锁存。
            return 0u; // 核心失败不能创建或放行控制任务。
        }
    }
#endif
    InitDiagnosticSetStage(INIT_STAGE_TASKS); // 后续失败属于任务创建阶段。
    if (!PlatformTaskBatchBegin()) // 暂停调度，保证句柄在任务运行前发布。
    {
        RobotStartupLatchFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                 INIT_FAILURE_DEPENDENCY, 0u); // 记录批次无法开始。
        return 0u; // 没有批次不能创建可能立即运行的任务。
    }
    const uint8_t created = OSTaskInit(); // 创建配置启用的任务并汇总结果。
    if (!created) // 恢复调度前先处理核心创建失败。
        RobotStartupLatchFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                 INIT_FAILURE_TASK_CREATE, 0u); // 失败时保持所有业务任务等待。
    if (!PlatformTaskBatchEnd()) // 恢复调度；已创建的业务任务仍等待放行。
    {
        RobotStartupLatchFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                 INIT_FAILURE_DEPENDENCY, 0u); // 记录批次恢复失败。
        return 0u; // 不得在调度状态不确定时放行。
    }
    if (!created) return 0u; // 核心创建失败时保持延迟状态。
    if (!BSP_WatchdogInitChecked(200u)) // 在允许等待的任务上下文切换运行窗口。
    {
        RobotStartupLatchFailure(INIT_STAGE_TASKS, INIT_MODULE_WATCHDOG,
                                 INIT_FAILURE_WATCHDOG_CONFIG, INIT_RESOURCE_WATCHDOG_RUNTIME);
        return 0u; // 窗口未确认时不能启动周期控制。
    }
    if (TaskMonitorReleaseRuntime() != TASK_MONITOR_RESULT_OK) // 从此刻开始计算首次期限。
    {
        RobotStartupLatchFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                 INIT_FAILURE_DEPENDENCY, 0u); // 长操作或故障未结束时保持关闭。
        return 0u; // 未完成放行时保留失能状态。
    }
    InitDiagnosticMarkReady(); // 启动完成不代表传感器数据已有效或控制已授权。
    return 1u; // 协调步骤完成，周期任务可按各自许可运行。
}
