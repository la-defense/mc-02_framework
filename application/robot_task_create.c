/** @file robot_task_create.c
 * @brief 按已确定的模块启动结果创建任务并报告失败。
 */
#include "robot_task_create.h"
#include "robot_task.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "platform_os.h"
#include "platform_uart.h"
#include "bsp_log.h"
#include "daemon.h"
#include "init_diagnostic.h"
#include <stddef.h>
#if !defined(MC02_PROFILE_BENCH_SAFE) && MC02_SELECTED_DRIVER_HT
#include "../modules/motor/HTmotor/ht_motor_task.h"
#endif

void StartLCDTASK(void const *argument);
PlatformTaskHandle_t insTaskHandle;
PlatformTaskHandle_t robotTaskHandle;
PlatformTaskHandle_t motorTaskHandle;
PlatformTaskHandle_t daemonTaskHandle;
PlatformTaskHandle_t uiTaskHandle;
PlatformTaskHandle_t lcdTaskHandle;

// 保留原任务名称、入口、栈字数和优先级；失败时句柄维持空值供后续检查。
static void CreateTask(const char *name, PlatformTaskEntry_t entry, uint32_t stack_words,
                       PlatformTaskPriority_t priority, PlatformTaskHandle_t *handle)
{
    const PlatformTaskConfig_t config = {
        .name = name, // 后端复制任务名称。
        .entry = entry, // 原来的常驻任务入口。
        .argument = NULL, // 这些入口均不使用参数。
        .stack_words = stack_words, // 沿用原 FreeRTOS 栈字数。
        .priority = priority, // 由平台后端映射为原有优先级。
    };
    *handle = NULL; // 每次创建前清空，避免旧句柄掩盖本次失败。
    (void)PlatformTaskCreateTracked(&config, handle); // 调用后由原检查集中报告失败。
}

#if MC02_FEATURE_HEATER
uint8_t RobotReportHeaterInitFailure(void)
{
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildMarkFailure(ROBOT_MODULE_HEATER) == 0u)
    {
        InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_INS, INIT_FAILURE_RESOURCE, 0u);
        return 0u; // 仅辅助功能受影响时保留其他模块。
    }
#endif
    InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS, INIT_FAILURE_RESOURCE, 0u);
    return 1u; // 核心失败由调用方立即关断并锁存。
}
#endif

uint8_t OSTaskInit(void)
{
    CreateTask("instask", StartINSTASK, 1024u, PLATFORM_TASK_ABOVE_NORMAL, &insTaskHandle); // 姿态读取需要较高优先级。

    // 裸板也要消费视觉等通信输入；只在机器人配置中运行电机控制。
#if MC02_FEATURE_COMM_DISPATCH
    CreateTask("motortask", StartMOTORTASK, 512u, PLATFORM_TASK_NORMAL, &motorTaskHandle); // 通信与电机任务。
#endif

    uint8_t uart_tx_created = PlatformUartStartTxTask(); // 尝试创建串口发送任务。
    uint8_t all_created = 1u; // 先假定核心任务均创建成功。

    // 日志是辅助功能时可降级；被核心依赖时创建失败须阻止启动。
    const BSPLogStatus_t log_status = BSPLogTaskInit(); // 获取日志任务创建结果。
#if defined(MC02_CONFIG_GENERATED)
    if (log_status != BSP_LOG_STATUS_OK && log_status != BSP_LOG_STATUS_QUEUED)
    {
        if (RobotBuildMarkFailure(ROBOT_MODULE_LOG) != 0u)
        {
            // 将依赖核心的日志失败记录为任务创建故障。
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LOG);
            all_created = 0u; // 向整车流程传递核心失败。
        }
        else
            // 辅助日志失败时记录降级，继续创建其他任务。
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LOG);
    }
#else
    (void)log_status; // 旧配置沿用原日志初始化行为。
#endif

    // 只有登记了独立事件来源时才创建工作线程；来源事件不能在监控任务里直接恢复模块。
    const DaemonStatus_t event_task_status = DaemonEventTaskInit();
    const uint8_t event_task_created = (uint8_t)(event_task_status == DAEMON_OK ||
                                                   event_task_status == DAEMON_NO_TASK_REQUIRED);

    // Daemon 内有状态检查与日志，需同时容纳日志行数组和各层调用。
    CreateTask("daemontask", StartDAEMONTASK, 512u, PLATFORM_TASK_NORMAL, &daemonTaskHandle); // 状态检查与恢复。

    CreateTask("robottask", StartROBOTTASK, 1024u, PLATFORM_TASK_NORMAL, &robotTaskHandle); // 整车控制与诊断。

#if MC02_FEATURE_UI
#if defined(MC02_CONFIG_GENERATED)
    // 依赖校验可能跳过未满足前置条件的辅助UI模块，此时不创建对应任务。
    if (RobotBuildModuleStarted(ROBOT_MODULE_UI))
    {
#endif
    CreateTask("uitask", StartUITASK, 512u, PLATFORM_TASK_NORMAL, &uiTaskHandle); // 裁判图形更新。
#if defined(MC02_CONFIG_GENERATED)
    }
#endif
#endif

#if MC02_FEATURE_LCD
#if defined(MC02_CONFIG_GENERATED)
    // 依赖校验跳过的可选LCD模块不创建任务。
    if (RobotBuildModuleStarted(ROBOT_MODULE_LCD))
    {
#endif
    CreateTask("lcdtask", StartLCDTASK, 1024u, PLATFORM_TASK_LOW, &lcdTaskHandle); // 显示任务沿用低优先级。
#if defined(MC02_CONFIG_GENERATED)
    }
#endif
#endif
    if (!event_task_created)
    {
        // 来源事件任务创建失败，记录具体资源并阻止核心启动。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DAEMON);
        all_created = 0u; // 汇总结果通知整车启动流程。
    }
    if (!uart_tx_created) {
        // 串口发送任务缺失时，保留失败原因供启动诊断查询。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
            INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UART_TX);
        all_created = 0u; // 核心任务缺失不允许继续启动。
    }
    if (insTaskHandle == NULL)
    {
        // 姿态任务没有句柄时记录创建失败。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_INS);
        all_created = 0u; // 姿态任务是当前构建的核心任务。
    }
#if MC02_FEATURE_COMM_DISPATCH
    if (motorTaskHandle == NULL)
    {
        // 已启用通信派发时，电机任务创建失败会阻断核心启动。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_MOTOR);
        all_created = 0u; // 向整车启动流程传递失败。
    }
#endif
    if (daemonTaskHandle == NULL)
    {
        // 来源监控任务缺失时记录对应资源。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DAEMON);
        all_created = 0u; // 不在缺少监控任务时启动控制。
    }
    if (robotTaskHandle == NULL)
    {
        // 整车协调任务缺失时记录对应资源。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_ROBOT);
        all_created = 0u; // 向上返回核心失败。
    }
#if MC02_FEATURE_UI
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_UI) && uiTaskHandle == NULL)
#else
    if (uiTaskHandle == NULL)
#endif
    {
#if defined(MC02_CONFIG_GENERATED)
        if (RobotBuildMarkFailure(ROBOT_MODULE_UI) != 0u)
        {
            // UI 影响核心依赖时记录致命创建失败。
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
            all_created = 0u; // 阻止整车核心启动。
        }
        else
            // UI 仅为辅助模块时，记录降级并继续启动其他任务。
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
#else
        // 旧配置没有模块依赖图，沿用失败即阻断的行为。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
        all_created = 0u; // 将旧配置的创建失败传回整车。
#endif
    }
#endif
#if MC02_FEATURE_LCD
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_LCD) && lcdTaskHandle == NULL)
#else
    if (lcdTaskHandle == NULL)
#endif
    {
#if defined(MC02_CONFIG_GENERATED)
        if (RobotBuildMarkFailure(ROBOT_MODULE_LCD) != 0u)
        {
            // LCD 被核心依赖时，创建失败须阻断整车。
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
            all_created = 0u; // 传递核心失败。
        }
        else
            // LCD 是辅助模块时记录降级，其他任务继续启动。
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
#else
        // 旧配置没有模块依赖图，保留失败即阻断的处理。
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
            INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
        all_created = 0u; // 将旧配置的创建失败传回整车。
#endif
    }
#endif

    if (all_created)
    {
#if !defined(MC02_PROFILE_BENCH_SAFE) && MC02_SELECTED_DRIVER_HT
        if (!HTMotorControlInit())
        {
            // HT 控制任务创建失败时，沿用电机任务失败的整车锁存路径。
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_MOTOR);
            all_created = 0u;
        }
#endif
    }
    return all_created;
}
