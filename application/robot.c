/** @file robot.c
* @brief 启动板级资源、应用和任务，失败时统一关闭输出，并提供整车周期入口。
*/
#include "bsp_init.h"
#include "platform_sync.h"
#include "config/build_selection.h"
#if MC02_FEATURE_VISION
#include "master_process.h"
#endif
#include "robot.h"
#include "robot_param_guard.h"
#include "robot_control_tasks.h"
#include "robot_def.h"
#include "robot_task_create.h"
#include "robot_monitor_plan.h"
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
#include "platform_feature_startup.h"
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

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if MC02_FEATURE_COMMAND || defined(ONE_BOARD) || defined(GIMBAL_BOARD)
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
        return 1u;
    case ROBOT_MODULE_LCD:
        return PlatformFeatureLcdReady(); // SPI1 失败时按模块重要性处理，不创建 LCD 任务。
    case ROBOT_MODULE_IMU:
        return (uint8_t)(PlatformFeatureImuReady() && INS_ConfigureSensor(config->imu_sensor)); // SPI2 就绪后才允许注册板载 IMU。
    case ROBOT_MODULE_HEATER:
#if MC02_FEATURE_HEATER
        return (uint8_t)(PlatformFeatureHeaterReady() &&
                         IMUHeaterConfigure(config->imu_heater_pwm, config->imu_heater_supply)); // PWM 失败按模块依赖处理。
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
    // 早期步骤允许硬件中断；共享数据各自由短临界区保护。
    InitDiagnosticBegin(INIT_STAGE_BSP);
    
    const uint8_t bsp_ready = BSPInit(); // 保存基础保护结果，不能将失败当作已准备好。
    RobotSafetyInit();
    if (!bsp_ready)
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_WATCHDOG,
                                    INIT_FAILURE_WATCHDOG_CONFIG,
                                    INIT_RESOURCE_WATCHDOG_STARTUP); // 记录启动窗口未就绪。
        goto init_failed; // 使用整车既有的关断和失能路径。
    }
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
    BSP_ResetRecoveryState_t recovery_state = {0};
    const uint8_t recovery_state_valid = BSP_ResetRecoveryGetState(&recovery_state);
    if (recovery_state_valid == 0u || recovery_state.locked_out != 0u)
        RobotFaultReport(ROBOT_FAULT_RESET_LOCKOUT);
    if (recovery_state_valid != 0u && recovery_state.manual_estop_latched != 0u)
        EstopRequest(ESTOP_REASON_MANUAL);
    #if defined(MC02_CONFIG_GENERATED)
    if (RobotConfigureTaskMonitor(RobotBuildGetPlan()) != TASK_MONITOR_RESULT_OK)
#else
    if (RobotConfigureTaskMonitor(NULL) != TASK_MONITOR_RESULT_OK)
#endif
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_DEPENDENCY, 0u);
        goto init_failed;
    }
    if (TaskMonitorDeferRuntime() != TASK_MONITOR_RESULT_OK)
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_DEPENDENCY, 0u); // 无法推迟监控时禁止启动。
        goto init_failed; // 不让周期任务在设备准备前运行。
    }
    (void)VehicleHealthInit();
    (void)VehicleHealthDiagnosticInit();
    const ParamLongOperationGuard_t param_guard = {
        .begin = RobotParamFlashOperationBegin,
        .end = RobotParamFlashOperationEnd,
    };
    ParamSetLongOperationGuard(&param_guard);

#if defined(MC02_PROFILE_ROBOT)
    // 日志诊断处理函数须在调度器启动前登记；实际话题仍由后续应用步骤建立。
    (void)RobotTopicsDiagnosticInit();
#endif
    InitDiagnosticSetStage(INIT_STAGE_FREERTOS); // 基础条件已准备，等待调度器启动。
    return 1u; // 业务设备在默认任务中继续初始化。

init_failed:
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE); // 保存早期核心初始化失败。
    RobotSafetyRequestDisable(); // 撤销业务运行许可。
    BSP_SafetyStartupLatchOutputsOff(); // 立即关闭受控输出。
    return 0u; // main 保留已有的失败诊断循环。
}

uint8_t RobotInitializeApplications(void)
{
    const Robot_Config_s *robot_config = NULL; // 只读取启动时确认的车型配置。
    const RobotConfigStatus_t config_status = RobotConfigSelect(ROBOT_CONFIG_SUPPORTED_ID, &robot_config);
    if (config_status != ROBOT_CONFIG_OK || robot_config == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_DEPENDENCY, (uint16_t)config_status); // 配置失效时记录原因。
        goto init_failed; // 不使用缺失或错误的配置。
    }
#if !defined(MC02_CONFIG_GENERATED)
    (void)RobotAlarmInit(robot_config->alarm_config); // 保留旧主机组合的告警初始化。
#endif

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
#endif

#undef ROBOT_INIT_CHECK
    return 1u; // 应用配置已检查，协调者继续初始化 INS。

// 保留首次初始化失败并立即失能，协调者负责维持诊断。
init_failed:
#undef ROBOT_INIT_CHECK
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestDisable();
    BSP_SafetyStartupLatchOutputsOff();
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
