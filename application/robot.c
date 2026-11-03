/** @file robot.c
* @brief 启动板级资源、应用和任务，失败时统一关闭输出，并提供整车周期入口。
*/
#include "bsp_init.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "config/build_selection.h"
#if MC02_FEATURE_VISION
#include "master_process.h"
#endif
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "robot_safety.h"
#include "config.h"
#include "robot_alarm.h"
#include "alarm_startup.h"
#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "bsp_param.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#endif
#include "init_diagnostic.h"
#include "vehicle_health.h"
#include "bsp_safety.h"
#include "bsp_reset_recovery.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#include "topics/robot_topics.h"
#if MC02_FEATURE_REFEREE
#include "rm_referee.h"
#if MC02_BENCH_REFEREE_DIAGNOSTICS
#include "referee_bench.h"
#endif
#endif
#if MC02_FEATURE_REMOTE
#include "remote_control.h"
#if MC02_BENCH_RC_DIAGNOSTICS
#include "remote_bench.h"
#endif
#endif
#if MC02_FEATURE_CHASSIS
#include "chassis.h"
#endif
#if MC02_FEATURE_GIMBAL
#include "gimbal.h"
#endif
#if MC02_FEATURE_SHOOTER
#include "shoot.h"
#endif
#if MC02_FEATURE_UI
#include "referee_task.h"
#endif
#endif
#if defined(MC02_PROFILE_ROBOT)
#include "topics/robot_topics_diag.h"
#endif
#if MC02_BENCH_RC_DIAGNOSTICS
#include "remote_bench.h"
#endif

#if MC02_BENCH_REFEREE_DIAGNOSTICS
#include "referee_bench.h"
#endif

// 这些期限沿用现有监控间隔；首次任务启动另留100ms，避免调度器刚启动时误报。
static const TaskMonitorTaskConfig_t robot_task_monitor_config[] = {
    {.id = TASK_MONITOR_INS, .name = "INS", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u,
     .startup_grace_ms = 100u},
#if MC02_FEATURE_COMM_DISPATCH
    {.id = TASK_MONITOR_MOTOR, .name = "MOTOR", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u,
     .startup_grace_ms = 100u},
#endif
    {.id = TASK_MONITOR_ROBOT, .name = "ROBOT", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 5u, .deadline_ms = 20u,
     .startup_grace_ms = 100u},
    {.id = TASK_MONITOR_DAEMON, .name = "DAEMON", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 10u, .deadline_ms = 30u,
     .startup_grace_ms = 100u},
    {.id = TASK_MONITOR_UART_TX, .name = "UART_TX", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 30u,
     .startup_grace_ms = 100u},
};

// 启用温控时先锁止加热，再申请 INS 的 Flash 等待时间；失败时结束相应长操作。
static uint8_t RobotParamFlashOperationBegin(void)
{
#if MC02_FEATURE_HEATER
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH))
        return 0u;
#endif
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH))
    {
#if MC02_FEATURE_HEATER
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
#endif
        return 0u;
    }
    return 1u;
}

// Flash 自身成功还不够，任务监控也必须按时结束；启用温控时结果还用于加热恢复判定。
static uint8_t RobotParamFlashOperationEnd(uint8_t success)
{
    uint8_t within_window = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                        TASK_MONITOR_LONG_OPERATION_FLASH);
    uint8_t operation_success = (uint8_t)(success && within_window);
#if MC02_FEATURE_HEATER
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, operation_success);
#endif
    return operation_success;
}

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
#include "topics/robot_can_bridge.h"
#endif

#if defined(MC02_CONFIG_GENERATED)
// 仅执行车型计划列出的模块初始化，并返回各模块实际启动结果。
static uint8_t RobotStartConfiguredModule(void *context, unsigned module)
{
    const Robot_Config_s *config = (const Robot_Config_s *)context;
    if (config == NULL)
        return 0u;
    switch (module)
    {
    case ROBOT_MODULE_SAFETY:
    case ROBOT_MODULE_LOG:
    case ROBOT_MODULE_LCD:
        return 1u;
    case ROBOT_MODULE_IMU:
        return INS_ConfigureSensor(config->imu_sensor); // 先核对板级绑定，实际采样在 INS 任务初始化。
    case ROBOT_MODULE_HEATER:
#if MC02_FEATURE_HEATER
        return IMUHeaterConfigure(config->imu_heater_pwm, config->imu_heater_supply); // 缺少资源时交给模块依赖处理。
#else
        return 0u;
#endif
    case ROBOT_MODULE_COMMUNICATION:
#if defined(MC02_PROFILE_ROBOT)
        return RobotTopicsInit();
#else
        return 1u;
#endif
    case ROBOT_MODULE_COMMAND:
#if MC02_FEATURE_COMMAND
        return RobotCMDInit();
#else
        return 0u;
#endif
    case ROBOT_MODULE_CHASSIS:
#if MC02_FEATURE_CHASSIS
        return ChassisInit();
#else
        return 0u;
#endif
    case ROBOT_MODULE_GIMBAL:
#if MC02_FEATURE_GIMBAL
        return GimbalInit();
#else
        return 0u;
#endif
    case ROBOT_MODULE_SHOOTER:
#if MC02_FEATURE_SHOOTER
        return ShootInit();
#else
        return 0u;
#endif
    case ROBOT_MODULE_REFEREE:
#if MC02_FEATURE_REFEREE
#if MC02_BENCH_REFEREE_DIAGNOSTICS
        return RefereeBenchInit(config->referee_uart);
#else
        return (uint8_t)(RefereeInit(config->referee_uart, NULL) == REFEREE_OK);
#endif
#else
        return 0u;
#endif
    case ROBOT_MODULE_VISION:
#if MC02_FEATURE_VISION
        return (uint8_t)(VisionInit(config->vision_uart) != NULL);
#else
        return 0u;
#endif
    case ROBOT_MODULE_REMOTE:
#if MC02_FEATURE_REMOTE
#if MC02_BENCH_RC_DIAGNOSTICS
        return RemoteBenchInit(config->remote_uart);
#else
        return (uint8_t)(RemoteControlInit(config->remote_uart) != NULL);
#endif
#else
        return 0u;
#endif
    case ROBOT_MODULE_ALARM:
#if MC02_FEATURE_ALARM
        return RobotStartAlarmConfigured(config->alarm_config, RobotBuildGetPlan());
#else
        return 0u;
#endif
    case ROBOT_MODULE_UI:
#if MC02_FEATURE_UI
        if (config->referee_ui_config == NULL)
            return 0u;
        return (uint8_t)(UITaskInit(config->referee_ui_config) == UI_OK);
#else
        return 0u;
#endif
    case ROBOT_MODULE_SUPERCAP:
        return 0u;
    default:
        return 0u;
    }
}
#endif


uint8_t RobotInit(void)
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    const PlatformIrqState_t previous_primask = PlatformIrqSaveAndDisable(); // 保留进入前的屏蔽状态。
    InitDiagnosticBegin(INIT_STAGE_BSP);
    
    BSPInit();
    RobotSafetyInit();
    const Robot_Config_s *robot_config = NULL;
    const RobotConfigStatus_t config_status = RobotConfigSelect(
        ROBOT_CONFIG_SUPPORTED_ID, &robot_config);
    if (config_status != ROBOT_CONFIG_OK || robot_config == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_DEPENDENCY,
                                    (uint16_t)config_status);
        goto init_failed;
    }
#if !defined(MC02_CONFIG_GENERATED)
    // 兼容旧主机测试构建；正式固件由配置启动器按依赖顺序初始化。
    (void)RobotAlarmInit(robot_config->alarm_config);
#endif
    BSP_ResetRecoveryState_t recovery_state = {0};
    const uint8_t recovery_state_valid = BSP_ResetRecoveryGetState(&recovery_state);
    if (recovery_state_valid == 0u || recovery_state.locked_out != 0u)
        RobotFaultReport(ROBOT_FAULT_RESET_LOCKOUT);
    if (recovery_state_valid != 0u && recovery_state.manual_estop_latched != 0u)
        EstopRequest(ESTOP_REASON_MANUAL);
    if (TaskMonitorInitConfigured(robot_task_monitor_config,
                                  (uint32_t)(sizeof(robot_task_monitor_config) /
                                             sizeof(robot_task_monitor_config[0]))) !=
        TASK_MONITOR_RESULT_OK)
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_DEPENDENCY, 0u);
        goto init_failed;
    }
    (void)VehicleHealthInit();
    (void)VehicleHealthDiagnosticInit();
    const ParamLongOperationGuard_t param_guard = {
        .begin = RobotParamFlashOperationBegin,
        .end = RobotParamFlashOperationEnd,
    };
    ParamSetLongOperationGuard(&param_guard);

#define ROBOT_INIT_CHECK(condition, stage, module, reason, resource)              \
    do                                                                            \
    {                                                                             \
        if (!(condition))                                                         \
        {                                                                         \
            if (!InitDiagnosticIsFailed())                                        \
                InitDiagnosticReportFailure((stage), (module), (reason), (resource)); \
            goto init_failed;                                                    \
        }                                                                         \
    } while (0)

#if defined(MC02_CONFIG_GENERATED)
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    const RobotModuleStatus_t module_status = RobotBuildStart(
        RobotStartConfiguredModule, (void *)robot_config);
    if (module_status != ROBOT_MODULE_OK)
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_DEPENDENCY,
                                    robot_module_startup.failed_module);
        goto init_failed;
    }
    for (unsigned module = 0u; module < ROBOT_MODULE_COUNT; ++module)
    {
        const uint32_t bit = ROBOT_MODULE_BIT(module);
        if ((robot_module_startup.failed & bit) != 0u ||
            (robot_module_startup.blocked & bit) != 0u)
            InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                         INIT_FAILURE_DEPENDENCY, module);
    }
#else
#if MC02_FEATURE_IMU
    ROBOT_INIT_CHECK(INS_ConfigureSensor(robot_config->imu_sensor), INIT_STAGE_APPLICATION,
                     INIT_MODULE_INS, INIT_FAILURE_DEPENDENCY, 0u);
#endif
#if MC02_FEATURE_HEATER
    ROBOT_INIT_CHECK(IMUHeaterConfigure(robot_config->imu_heater_pwm, robot_config->imu_heater_supply),
                     INIT_STAGE_APPLICATION, INIT_MODULE_INS, INIT_FAILURE_DEPENDENCY, 0u);
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    ROBOT_INIT_CHECK(RobotCMDInit(), INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                     INIT_FAILURE_DEPENDENCY, 1u);
    ROBOT_INIT_CHECK(GimbalInit(), INIT_STAGE_APPLICATION, INIT_MODULE_GIMBAL,
                     INIT_FAILURE_DEPENDENCY, 0u);
    ROBOT_INIT_CHECK(ShootInit(), INIT_STAGE_APPLICATION, INIT_MODULE_SHOOTER,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif

#if defined(MC02_PROFILE_BENCH_SAFE)
#if MC02_FEATURE_VISION
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    ROBOT_INIT_CHECK(VisionInit(robot_config->vision_uart) != NULL, INIT_STAGE_APPLICATION, INIT_MODULE_VISION,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#if MC02_BENCH_REFEREE_DIAGNOSTICS
    ROBOT_INIT_CHECK(RefereeBenchInit(robot_config->referee_uart), INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#if MC02_BENCH_RC_DIAGNOSTICS
    ROBOT_INIT_CHECK(RemoteBenchInit(robot_config->remote_uart), INIT_STAGE_APPLICATION, INIT_MODULE_REMOTE,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    ROBOT_INIT_CHECK(ChassisInit(), INIT_STAGE_APPLICATION, INIT_MODULE_CHASSIS,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#endif

#if defined(MC02_PROFILE_ROBOT)
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)
    ROBOT_INIT_CHECK(RobotCanBridgeInit(&robot_config->can_bridge), INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                     INIT_FAILURE_DEPENDENCY, 2u);
#endif
    // 话题统计只用于调试；注册失败不改变控制任务的启动结果。
    (void)RobotTopicsDiagnosticInit();
#endif

    InitDiagnosticSetStage(INIT_STAGE_TASKS);
    ROBOT_INIT_CHECK(OSTaskInit(), INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                     INIT_FAILURE_TASK_CREATE, 0u);

    // 所有任务创建完成后再启动IWDG, 由Daemon任务中的TaskMonitorTick统一喂狗
    BSP_WatchdogInit(200);

    InitDiagnosticSetStage(INIT_STAGE_FREERTOS);
#undef ROBOT_INIT_CHECK
    PlatformIrqRestore(previous_primask); // 成功后恢复调用前的中断状态。
    return 1u;

// 保留首次初始化失败，失能并恢复调用前的中断状态，让 main 继续运行诊断。
init_failed:
#undef ROBOT_INIT_CHECK
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestDisable();
    BSP_SafetyStartupLatchOutputsOff();
    PlatformIrqRestore(previous_primask); // 输出关断后才恢复原中断状态。
    return 0u;
}

void RobotTask()
{
    // 已有启动失败时不再运行任何控制应用，只维护可读取的诊断结果。
    if (InitDiagnosticIsFailed())
    {
        InitDiagnosticHeartbeat();
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        RobotSafetyRequestDisable();
        RobotSafetyUpdate();
        (void)VehicleHealthProcess();
        return;
    }
#if defined(MC02_PROFILE_BENCH_SAFE)
#if MC02_BENCH_RC_DIAGNOSTICS
    RemoteBenchTask();
#else
    RobotSafetyUpdate();
#endif
#if MC02_BENCH_REFEREE_DIAGNOSTICS
    RefereeBenchTask();
#endif
#if MC02_FEATURE_VISION
#if defined(MC02_CONFIG_GENERATED)
    // 依赖计划未接受启动的模块可能尚未初始化，不能仅凭编译开关调用其业务入口。
    if (RobotBuildModuleStarted(ROBOT_MODULE_VISION))
        VisionSend();
#else
    VisionSend();
#endif
#endif
#else
#if defined(MC02_CONFIG_GENERATED)
// 只调度启动计划已接受的模块；被依赖失败阻断或初始化失败的模块保持不调用。
#if MC02_FEATURE_COMMAND
    if (RobotBuildModuleStarted(ROBOT_MODULE_COMMAND))
        RobotCMDTask();
#endif
#if MC02_FEATURE_GIMBAL
    if (RobotBuildModuleStarted(ROBOT_MODULE_GIMBAL))
        GimbalTask();
#endif
#if MC02_FEATURE_SHOOTER
    if (RobotBuildModuleStarted(ROBOT_MODULE_SHOOTER))
        ShootTask();
#endif
#if MC02_FEATURE_CHASSIS
    if (RobotBuildModuleStarted(ROBOT_MODULE_CHASSIS))
        ChassisTask();
#endif
#else
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif
#endif
#endif

    (void)VehicleHealthProcess();
    (void)BSP_ResetRecoveryObserveControl(PlatformTimeNowMs(), RobotSafetyIsReady(),
                                          TaskMonitorAllCoreHealthyNow());

}
