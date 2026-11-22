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
#include "robot_startup.h"
#include "robot_param_guard.h"
#include "robot_power.h"
#include "robot_can_guard.h"
#include "robot_can_runtime.h"
#if defined(MC02_CONFIG_GENERATED) && (defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)) && defined(ONE_BOARD) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE && MC02_SELECTED_DRIVER_DJI
#include "robot_can_recovery_coordinator.h"
#include "robot_control_feedback_runtime.h"
#endif
#include "robot_can_capacity.h"
#include "robot_can_requirements.h"
#include "platform_can.h"
#include "robot_control_tasks.h"
#include "robot_def.h"
#include "robot_task_create.h"
#include "robot_monitor_plan.h"
#include "robot_safety.h"
#include "config.h"
#include "robot_alarm.h"
#include "alarm_startup.h"
#include "log_startup.h"
#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "bsp_param.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#include "bsp_adc.h"
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
#if defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
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

#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
#include "commissioning_chassis_cmd.h"
#include "chassis/chassis_motion_diag.h"
#elif MC02_FEATURE_COMMAND || defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "robot_cmd.h"
#endif
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
#include "topics/robot_can_bridge.h"
#endif

#if defined(MC02_CONFIG_GENERATED)
typedef struct {
    const Robot_Config_s *config; // 启动时只读的车型配置。
    uint32_t can_rejected; // 容量不足而拒绝的辅助模块位图。
    uint8_t rollback_failed; // 已发生不能安全清理的资源故障。
} RobotStartContext_t;

// 仅执行车型计划列出的模块初始化，并返回各模块实际启动结果。
static uint8_t RobotStartConfiguredModule(void *context, unsigned module)
{
    RobotStartContext_t *start = (RobotStartContext_t *)context; // 同时读取车型、预检及回退结果。
    if (start == NULL || start->config == NULL)
        return 0u; // 缺少配置不能启动设备。
    const Robot_Config_s *config = start->config; // 模块继续使用只读配置。
    if (start->rollback_failed) return 0u; // 回退异常后不再注册其他业务设备。
    if ((start->can_rejected & ROBOT_MODULE_BIT(module)) != 0u)
        return 0u; // 被拒绝的辅助模块不得部分注册设备。
    if (!RobotPowerModuleReady(module))
        return 0u; // 所需电源未准备时不注册该模块的设备。
    switch (module)
    {
    case ROBOT_MODULE_SAFETY:
        return 1u;
    case ROBOT_MODULE_LOG:
        return RobotStartLogConfigured(RobotBuildGetPlan()); // RTT 失败按核心依赖决定是否继续。
    case ROBOT_MODULE_LCD:
        return PlatformFeatureLcdReady(); // SPI1 失败时按模块重要性处理，不创建 LCD 任务。
    case ROBOT_MODULE_IMU:
        return (uint8_t)(PlatformFeatureImuReady() && INS_ConfigureSensor(config->imu_sensor)); // SPI2 就绪后才允许注册板载 IMU。
    case ROBOT_MODULE_HEATER:
#if MC02_FEATURE_HEATER
        return (uint8_t)(PlatformFeatureHeaterReady() && BSP_ADCIsStarted() &&
                         IMUHeaterConfigure(config->imu_heater_pwm, config->imu_heater_supply)); // PWM 失败按模块依赖处理。
#else
        return 0u;
#endif
    case ROBOT_MODULE_COMMUNICATION:
#if defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)
        if (!PlatformFeatureCanReady(config->can_bridge.can_bus)) return 0u; // 双板桥接必须先有可用总线。
#endif
        return RobotTopicsInit();
#else
        return 1u;
#endif
    case ROBOT_MODULE_COMMAND:
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
        return CommissioningChassisCmdInit(&config->manual_input); // 只登记人工底盘命令。
#elif MC02_FEATURE_COMMAND
        return RobotCMDInit();
#else
        return 0u;
#endif
    case ROBOT_MODULE_CHASSIS:
#if MC02_FEATURE_CHASSIS
        if (!PlatformFeatureCanReady(config->chassis_param.chassis_motor_config.can_init_config.can_bus))
            return 0u; // 轮电机总线失败时不分配任何电机实例。
#if MC02_FEATURE_SUPERCAP
        if (!PlatformFeatureCanReady(PLATFORM_CAN_BUS_2)) return 0u; // 旧电容绑定的总线必须可用。
#endif
        if (ChassisInit()) return 1u; // 完整注册后才接受底盘模块。
        if (ChassisInitRollbackFailed())
        {
            start->rollback_failed = 1u; // 无法安全清理时禁止后续模块注册。
            RobotStartupRollbackFailure(INIT_MODULE_CHASSIS); // 立即锁存故障并关断输出。
        }
        return 0u; // 正常回退时由模块计划判断故障影响。
#else
        return 0u;
#endif
    case ROBOT_MODULE_GIMBAL:
#if MC02_FEATURE_GIMBAL
        if (!PlatformFeatureCanReady(config->gimbal_param.yaw_motor_config.can_init_config.can_bus) ||
            !PlatformFeatureCanReady(config->gimbal_param.pitch_motor_config.can_init_config.can_bus))
            return 0u; // 任一电机总线不可用时不部分注册云台。
        if (GimbalInit()) return 1u; // 完整注册后才接受云台模块。
        if (GimbalInitRollbackFailed())
        {
            start->rollback_failed = 1u; // 回退失败不能作为普通辅助故障处理。
            RobotStartupRollbackFailure(INIT_MODULE_GIMBAL); // 立即记录故障并失能、关断。
        }
        return 0u; // 正常回退后由模块计划决定故障影响。
#else
        return 0u;
#endif
    case ROBOT_MODULE_SHOOTER:
#if MC02_FEATURE_SHOOTER
        if (!PlatformFeatureCanReady(config->shoot_param.friction_motor_config.can_init_config.can_bus) ||
            !PlatformFeatureCanReady(config->shoot_param.loader_motor_config.can_init_config.can_bus))
            return 0u; // 任一电机总线不可用时不部分注册发射机构。
        if (ShootInit()) return 1u; // 完整注册后才接受模块。
        if (ShootInitRollbackFailed())
        {
            start->rollback_failed = 1u; // 无法清理时不能按普通辅助故障继续。
            RobotStartupRollbackFailure(INIT_MODULE_SHOOTER); // 记录故障并立即失能、关断。
        }
        return 0u; // 普通失败可由模块计划按依赖决定降级。
#else
        return 0u;
#endif
    case ROBOT_MODULE_REFEREE:
#if MC02_FEATURE_REFEREE
        if (!PlatformFeatureUartReady(config->referee_uart)) return 0u; // 端口失败时不注册裁判接收。
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
#if defined(VISION_USE_UART)
        if (!PlatformFeatureUartReady(config->vision_uart)) return 0u; // UART 视觉需要所选串口。
#endif
        return (uint8_t)(VisionInit(config->vision_uart) != NULL);
#else
        return 0u;
#endif
    case ROBOT_MODULE_REMOTE:
#if MC02_FEATURE_REMOTE
        if (!PlatformFeatureUartReady(config->remote_uart)) return 0u; // 端口失败时不注册遥控接收。
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


uint8_t RobotRecheckCanAfterDiagnostics(void)
{
#if defined(MC02_CONFIG_GENERATED)
    const Robot_Config_s *config = NULL; // 重新读取启动时的只读车型配置。
    if (RobotConfigSelect(ROBOT_CONFIG_SUPPORTED_ID, &config) != ROBOT_CONFIG_OK || config == NULL)
        return 0u; // 配置不可用时不能判断总线依赖。
    RobotCanFilterDemand_t demands[8] = {0}; // 复用启动预检中的设备清单。
    const uint8_t count = RobotCollectCanDemands(config, demands); // 只包含当前启用设备。
    RobotCanDependency_t dependencies[8] = {0}; // 隔离检查只需要模块和总线。
    for (uint8_t i = 0u; i < count; ++i)
        dependencies[i] = (RobotCanDependency_t){demands[i].module, demands[i].bus}; // 不重复维护绑定。
    if (!RobotCanGuardCheck(dependencies, count)) return 0u; // 核心依赖失败时禁止后续任务。
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
    if (!RobotBuildModuleStarted(ROBOT_MODULE_COMMUNICATION) && !RobotCanBridgeStopStartup())
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_RESOURCE, 2u); // 端点无法安全关闭，保留核心失败原因。
        return 0u; // 接收端点清理异常时不能放行任务。
    }
#endif
    uint8_t released = 1u; // 所有辅助清理都要尝试，不能被短路条件跳过。
#if MC02_FEATURE_SHOOTER
    if (!RobotBuildModuleStarted(ROBOT_MODULE_SHOOTER) && !ShootReleaseStartupDevices()) released = 0u; // 清除已降级发射机构的电机登记。
#endif
#if MC02_FEATURE_GIMBAL
    if (!RobotBuildModuleStarted(ROBOT_MODULE_GIMBAL) && !GimbalReleaseStartupDevices()) released = 0u; // 姿态或总线失败也可能阻止云台。
#endif
#if MC02_FEATURE_CHASSIS
    if (!RobotBuildModuleStarted(ROBOT_MODULE_CHASSIS) && !ChassisReleaseStartupDevices()) released = 0u; // 保留其他成功模块的实例。
#endif
    if (!released)
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_RESOURCE, 3u); // 清理失败不能作为普通辅助降级继续。
    return released; // 成功清理后才允许创建周期任务。
#else
    return 1u; // 旧主机组合没有车型生成的依赖计划。
#endif
}

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
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    const PlatformPeriodicConfig_t *command_timing = RobotGetTaskPeriod(ROBOT_PERIODIC_DIAGNOSTIC); // 整车任务也是受限底盘的命令发布者。
    if (!RobotCommissioningChassisInputFitsPeriod(&robot_config->manual_input, command_timing->period_us))
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_TIMING, INIT_RESOURCE_TASK_ROBOT); // 人工期限短于命令拍点，不能进入设备启动。
        goto init_failed;
    }
    const PlatformPeriodicConfig_t *dispatch_timing = RobotGetTaskPeriod(ROBOT_PERIODIC_COMM); // 电机计算和发送共用此任务的执行预算。
    if (!RobotCommissioningChassisTimingFits(&robot_config->manual_input, command_timing,
            &robot_config->control_tasks[ROBOT_CONTROL_TASK_CHASSIS].timing, &robot_config->motor_periods, dispatch_timing))
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_TIMING, INIT_RESOURCE_TASK_CHASSIS_PIPELINE); // 各段组合超出原期限时保持失能。
        goto init_failed;
    }
    const RobotModulePlan_t *commissioning_modules = RobotBuildGetPlan(); // 与设备启动共享车型清单。
    const RobotControlPolicy_t commissioning_policy = {
        .enabled_functions = ROBOT_CONTROL_CHASSIS, // 仅允许轮组取得许可。
        .configured = 1u, // 车型配置读取成功后才按此启用策略。
        .formal_autonomous = 0u, // 受限调试没有自主控制资格。
        .commissioning = 1u, // 策略只接受人工来源。
    };
    RobotCanMotorPlan_t wheel_feedback = {0}; // 在注册设备前准备四轮身份。
    if (!RobotCanMotorPlanBuild(robot_config, commissioning_modules, &wheel_feedback) ||
        !RobotControlFeedbackStart(&wheel_feedback) ||
        !RobotSafetyConfigureControl(&commissioning_policy) ||
        !RobotCanRecoveryStart(robot_config, commissioning_modules))
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_DEPENDENCY, 0u); // 禁止未完整绑定的人工许可进入设备启动。
        goto init_failed;
    }
#endif
    BSP_ResetRecoveryState_t recovery_state = {0};
    const uint8_t recovery_state_valid = BSP_ResetRecoveryGetState(&recovery_state);
    if (recovery_state_valid == 0u || recovery_state.locked_out != 0u)
        RobotFaultReport(ROBOT_FAULT_RESET_LOCKOUT);
    if (recovery_state_valid != 0u && recovery_state.manual_estop_latched != 0u)
        EstopRequest(ESTOP_REASON_MANUAL);
#if defined(MC02_CONFIG_GENERATED)
    const RobotModulePlan_t *task_modules = RobotBuildGetPlan(); // 监控和启动检查使用相同模块选择。
#else
    const RobotModulePlan_t *task_modules = NULL; // 旧配置保留默认启用组合。
#endif
#if MC02_TASK_MOTOR_SPLIT
    if (RobotPrepareMotorTaskPeriods(&robot_config->motor_periods) != PLATFORM_PERIODIC_READY)
    {
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_TIMING, INIT_RESOURCE_TASK_MOTOR);
        goto init_failed; // 两个周期缺失或不可表示时，不启动硬件设备。
    }
#endif
#if MC02_TASK_FIXED_REGULAR_PERIODS
    static const uint16_t task_resources[ROBOT_PERIODIC_COUNT] = {
        [ROBOT_PERIODIC_DIAGNOSTIC] = INIT_RESOURCE_TASK_ROBOT, // 整车诊断。
        [ROBOT_PERIODIC_INS] = INIT_RESOURCE_TASK_INS, // 姿态采样。
        [ROBOT_PERIODIC_COMM] = INIT_RESOURCE_TASK_MOTOR, // 通信派发沿用任务资源编号。
        [ROBOT_PERIODIC_DAEMON] = INIT_RESOURCE_TASK_DAEMON, // 设备维护和监控。
#if MC02_BENCH_TIM5_DIAGNOSTICS
        [ROBOT_PERIODIC_TIM5_PROBE] = INIT_RESOURCE_TASK_TIM5_PROBE, // 专用定时自测。
#endif
    };
    RobotPeriodicTask_t failed_period; // 预检失败时取得具体任务。
    const PlatformPeriodicResult_t timing_status = RobotValidateTaskPeriods(task_modules, &failed_period);
    if (timing_status != PLATFORM_PERIODIC_READY) // 时基或硬件通道不满足要求时不启动任务。
    {
        const InitDiagnosticReason_e reason = timing_status == PLATFORM_PERIODIC_RESOURCE
            ? INIT_FAILURE_RESOURCE : INIT_FAILURE_TASK_TIMING; // 区分资源不可用与时间配置无效。
        InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS, reason, task_resources[failed_period]);
        goto init_failed; // 沿用现有启动失能路径。
    }
#endif
    RobotControlTimingFailure_t control_failure; // 保留联合预检返回的具体失败任务。
    if (RobotConfigureTaskMonitor(task_modules, robot_config->control_tasks, &control_failure) != TASK_MONITOR_RESULT_OK)
    {
#if MC02_TASK_CONTROL
        static const uint16_t control_resources[ROBOT_CONTROL_TASK_COUNT] = {
            [ROBOT_CONTROL_TASK_CHASSIS] = INIT_RESOURCE_TASK_CHASSIS, // 底盘独立任务。
            [ROBOT_CONTROL_TASK_GIMBAL] = INIT_RESOURCE_TASK_GIMBAL, // 云台独立任务。
            [ROBOT_CONTROL_TASK_SHOOTER] = INIT_RESOURCE_TASK_SHOOTER, // 发射模块独立任务。
        };
        if ((unsigned)control_failure.task < ROBOT_CONTROL_TASK_COUNT)
        {
            const InitDiagnosticReason_e reason = control_failure.reason == PLATFORM_PERIODIC_RESOURCE
                ? INIT_FAILURE_RESOURCE : INIT_FAILURE_TASK_TIMING; // 区分定时器通道不足与周期或预算无效。
            InitDiagnosticReportFailure(INIT_STAGE_BSP, INIT_MODULE_TASKS,
                                        reason, control_resources[control_failure.task]);
            goto init_failed; // 不通过时不进入设备初始化。
        }
#endif
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
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    (void)ChassisMotionDiagnosticInit(); // 受限底盘按需提供运动反馈，诊断失败不授予运行许可。
#endif
    const ParamLongOperationGuard_t param_guard = {
        .begin = RobotParamFlashOperationBegin,
        .end = RobotParamFlashOperationEnd,
    };
    ParamSetLongOperationGuard(&param_guard);

#if defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
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
    RobotCanFilterDemand_t can_demands[8] = {0}; // 预检不注册设备。
    const uint8_t can_count = RobotCollectCanDemands(robot_config, can_demands); // 汇总车型端点。
    RobotCanFilterCapacity_t can_capacity = {.endpoints = PlatformCanEndpointCapacity()}; // 全局端点上限。
    for (uint8_t i = 0u; i < can_count; ++i)
    {
        const PlatformCanBus_t bus = can_demands[i].bus; // 查询需求所属控制器。
        PlatformCanFilterCapacity_t actual = {0}; // 失败时容量保持为零。
        if (bus >= PLATFORM_CAN_BUS_1 && bus <= PLATFORM_CAN_BUS_3 &&
            PlatformCanGetFilterCapacity(bus, &actual))
        {
            const unsigned index = (unsigned)bus - (unsigned)PLATFORM_CAN_BUS_1; // 转成数组下标。
            can_capacity.standard[index] = actual.standard; // 记录标准 ID 槽数。
            can_capacity.extended[index] = actual.extended; // 扩展 ID 独立计数。
        }
    }
    RobotCanCapacityResult_t can_result = {0}; // 只有合法预检结果才可使用。
    const RobotModuleStatus_t can_status = RobotCanCapacityPlan(RobotBuildGetPlan(), can_demands,
                                                                 can_count, &can_capacity, &can_result);
    if (can_status != ROBOT_MODULE_OK)
    {
        const uint16_t resource = can_status == ROBOT_MODULE_CORE_FAILURE ? can_result.failed_core_module : (uint16_t)can_status; // 区分核心模块和无效计划。
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT, INIT_FAILURE_RESOURCE, resource); // 记录预检失败。
        goto init_failed; // 核心资源不足时不注册业务设备。
    }
#if defined(MC02_PROFILE_ROBOT) && defined(ONE_BOARD) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE && MC02_SELECTED_DRIVER_DJI
    RobotModulePlan_t feedback_modules = *RobotBuildGetPlan(); // 复制配置，保留原有启动依赖分类。
    const uint32_t motor_modules = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) |
        ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL) | ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER); // 已启用的机构都纳入身份核对。
    feedback_modules.required |= feedback_modules.enabled & motor_modules; // 此处核对设备身份，之后仍按原配置启动模块。
    RobotCanMotorPlan_t feedback_plan = {0}; // 先检查总线、型号和反馈编号，再建立实际反馈采集入口。
    ROBOT_INIT_CHECK(RobotCanMotorPlanBuild(robot_config, &feedback_modules, &feedback_plan) &&
                     RobotControlFeedbackStart(&feedback_plan) &&
                     RobotCanRecoveryStart(robot_config, RobotBuildGetPlan()), INIT_STAGE_APPLICATION,
                     INIT_MODULE_ROBOT, INIT_FAILURE_DEPENDENCY, 0u); // 核心配置或绑定有误，直接保持关断，尚未开启电源或注册设备。
#endif
#if defined(MC02_PROFILE_ROBOT) || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    if (!RobotCanRuntimeInstall(RobotBuildGetPlan(), can_demands, can_count))
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_RESOURCE, 0u); // 核心总线无法建立故障观察时记录启动失败。
        goto init_failed; // 不允许缺少故障通知的机器人固件进入控制。
    }
#endif
    RobotStartContext_t start_context = {.config = robot_config, .can_rejected = can_result.rejected_modules}; // 辅助失败由启动流程登记。
    const RobotPowerRailConfig_t power_rails[PLATFORM_POWER_COUNT] = {
        {MC02_POWER_24V_1_STAGE, MC02_POWER_24V_1_MASK}, // PC14：第一路 24V 的阶段及使用模块。
        {MC02_POWER_24V_2_STAGE, MC02_POWER_24V_2_MASK}, // PC13：第二路 24V 的阶段及使用模块。
        {MC02_POWER_5V_STAGE, MC02_POWER_5V_MASK}, // PC15：5V 的阶段及使用模块。
    };
    ROBOT_INIT_CHECK(RobotPowerBeginWithUnavailable(power_rails, RobotBuildGetPlan(), can_result.rejected_modules), INIT_STAGE_APPLICATION,
                     INIT_MODULE_ROBOT, INIT_FAILURE_RESOURCE, 0u); // 先校验计划并准备设备前电源。
    const RobotModuleStatus_t module_status = RobotBuildStartPhased(
        RobotPowerDeferredModules(), RobotPowerAfterDevices,
        RobotStartConfiguredModule, &start_context); // 独立模块完成后再准备设备后电源。
    if (module_status != ROBOT_MODULE_OK || start_context.rollback_failed)
    {
        if (!InitDiagnosticIsFailed()) // 不覆盖回退失败时已保存的具体原因。
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
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_COMMUNICATION) && !RobotCanBridgeInit(&robot_config->can_bridge))
    {
        if (RobotCanBridgeInitRollbackFailed())
        {
            RobotStartupRollbackFailure(INIT_MODULE_ROBOT); // 板间端点未能安全回退时整体失能。
            goto init_failed; // 不创建或放行业务任务。
        }
        if (RobotBuildMarkFailure(ROBOT_MODULE_COMMUNICATION)) // 桥接失败可能影响核心依赖。
        {
            InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT, INIT_FAILURE_RESOURCE, 2u); // 保存核心桥接失败。
            goto init_failed; // 不能放行核心通信缺失的车型。
        }
        InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT, INIT_FAILURE_RESOURCE, 2u); // 辅助通信失败只降级其依赖者。
    }
#else
    ROBOT_INIT_CHECK(RobotCanBridgeInit(&robot_config->can_bridge), INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                     INIT_FAILURE_DEPENDENCY, 2u); // 保留旧主机组合的启动行为。
#endif
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
#if MC02_FEATURE_GIMBAL && !MC02_TASK_GIMBAL
    if (RobotBuildModuleStarted(ROBOT_MODULE_GIMBAL))
        GimbalUpdateFeedback(); // 本轮命令读取刚采集的反馈，避免消耗整个 5 ms 周期。
#endif
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    if (RobotBuildModuleStarted(ROBOT_MODULE_COMMAND))
        CommissioningChassisCmdTask(); // 由同一个整车任务处理人工来源。
#elif MC02_FEATURE_COMMAND
    if (RobotBuildModuleStarted(ROBOT_MODULE_COMMAND))
        RobotCMDTask();
#endif
#if MC02_FEATURE_GIMBAL && !MC02_TASK_GIMBAL
    if (RobotBuildModuleStarted(ROBOT_MODULE_GIMBAL))
        GimbalTask();
#endif
#if MC02_FEATURE_SHOOTER && !MC02_TASK_SHOOTER
    if (RobotBuildModuleStarted(ROBOT_MODULE_SHOOTER))
        ShootTask();
#endif
#if MC02_FEATURE_CHASSIS && !MC02_TASK_CHASSIS
    if (RobotBuildModuleStarted(ROBOT_MODULE_CHASSIS))
        ChassisTask();
#endif
#else
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    GimbalUpdateFeedback(); // 兼容组合也按先更新反馈、再选择命令的顺序执行。
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
