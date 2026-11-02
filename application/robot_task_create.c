/** @file robot_task_create.c
 * @brief 按已确定的模块启动结果创建任务并报告失败。
 */
#include "robot_task_create.h"
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

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
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

/** @brief 创建INS、通信派发、电机、UART发送、Daemon、整车、LCD及当前配置所需UI任务。
* @return 1 表示核心任务创建成功；辅助任务失败已记录降级，核心任务失败返回0；不删除已创建任务。
* @note 由 RobotInit() 在调度器启动前调用一次。
 */
uint8_t OSTaskInit(void)
{
    CreateTask("instask", StartINSTASK, 1024u, PLATFORM_TASK_ABOVE_NORMAL, &insTaskHandle); // 姿态读取需要较高优先级。

    // 裸板也要消费视觉等通信输入；只在机器人配置中运行电机控制。
#if MC02_FEATURE_COMM_DISPATCH
    CreateTask("motortask", StartMOTORTASK, 512u, PLATFORM_TASK_NORMAL, &motorTaskHandle); // 通信与电机任务。
#endif

    uint8_t uart_tx_created = PlatformUartStartTxTask();
    uint8_t all_created = 1u;

    /* 日志配置为辅助时降级继续；若车型核心依赖日志，失败必须锁存启动失败。 */
    const BSPLogStatus_t log_status = BSPLogTaskInit();
#if defined(MC02_CONFIG_GENERATED)
    if (log_status != BSP_LOG_STATUS_OK && log_status != BSP_LOG_STATUS_QUEUED)
    {
        if (RobotBuildMarkFailure(ROBOT_MODULE_LOG) != 0u)
        {
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LOG);
            all_created = 0u;
        }
        else
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LOG);
    }
#else
    (void)log_status;
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
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DAEMON);
        all_created = 0u;
    }
    if (!uart_tx_created) {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
            INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UART_TX);
        all_created = 0u;
    }
    if (insTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_INS);
        all_created = 0u;
    }
#if MC02_FEATURE_COMM_DISPATCH
    if (motorTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_MOTOR);
        all_created = 0u;
    }
#endif
    if (daemonTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DAEMON);
        all_created = 0u;
    }
    if (robotTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_ROBOT);
        all_created = 0u;
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
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
            all_created = 0u;
        }
        else
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
#else
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
        all_created = 0u;
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
            InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                        INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
            all_created = 0u;
        }
        else
            InitDiagnosticReportDegraded(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                         INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
#else
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
            INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
        all_created = 0u;
#endif
    }
#endif

    if (all_created)
    {
#if !defined(MC02_PROFILE_BENCH_SAFE) && MC02_SELECTED_DRIVER_HT
        HTMotorControlInit(); // 没有注册HT电机则不会执行
#endif
    }
    return all_created;
}
