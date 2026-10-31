/** @file robot.c
* @brief 启动板级资源、应用和任务，失败时统一关闭输出，并提供整车周期入口。
*/
#include "bsp_init.h"
#include "main.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "robot_safety.h"
#include "config.h"
#include "robot_alarm.h"
#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "master_process.h"
#include "bsp_param.h"
#include "imu_heater.h"
#include "init_diagnostic.h"
#include "vehicle_health.h"
#include "bsp_safety.h"
#include "bsp_reset_recovery.h"
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
    {.id = TASK_MONITOR_MOTOR, .name = "MOTOR", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u,
     .startup_grace_ms = 100u},
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

// 写参数前先关闭加热，再申请 INS 的 Flash 等待时间；第二步失败时按失败结束加热长操作。
static uint8_t RobotParamFlashOperationBegin(void)
{
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH))
        return 0u;
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH))
    {
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
        return 0u;
    }
    return 1u;
}

// Flash 自身成功还不够，任务监控也必须按时结束；两个结果一起交给加热器决定是否等待新数据恢复。
static uint8_t RobotParamFlashOperationEnd(uint8_t success)
{
    uint8_t within_window = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                        TASK_MONITOR_LONG_OPERATION_FLASH);
    uint8_t operation_success = (uint8_t)(success && within_window);
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, operation_success);
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


uint8_t RobotInit(void)
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    const uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    InitDiagnosticBegin(INIT_STAGE_BSP);
    
    BSPInit();
    RobotSafetyInit();
    Robot_Config_s *robot_config = RobotConfigInit(ROBOT_ID);
    if (robot_config == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                    INIT_FAILURE_DEPENDENCY, 0u);
        goto init_failed;
    }
    // 告警输出属于辅助通道；LED 或蜂鸣器失败不阻断控制任务启动。
    (void)RobotAlarmInit(robot_config->alarm_config);
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
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    ROBOT_INIT_CHECK(VisionInit(&huart9) != NULL, INIT_STAGE_APPLICATION, INIT_MODULE_VISION,
                     INIT_FAILURE_DEPENDENCY, 0u);
#if MC02_BENCH_REFEREE_DIAGNOSTICS
    ROBOT_INIT_CHECK(RefereeBenchInit(&huart1), INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#if MC02_BENCH_RC_DIAGNOSTICS
    ROBOT_INIT_CHECK(RemoteBenchInit(&huart5), INIT_STAGE_APPLICATION, INIT_MODULE_REMOTE,
                     INIT_FAILURE_DEPENDENCY, 0u);
#endif
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    ROBOT_INIT_CHECK(ChassisInit(), INIT_STAGE_APPLICATION, INIT_MODULE_CHASSIS,
                     INIT_FAILURE_DEPENDENCY, 0u);
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
    __set_PRIMASK(previous_primask);
    return 1u;

// 保留首次初始化失败，失能并恢复调用前的中断状态，让 main 继续运行诊断。
init_failed:
#undef ROBOT_INIT_CHECK
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestDisable();
    BSP_SafetyStartupLatchOutputsOff();
    __set_PRIMASK(previous_primask);
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
    VisionSend();
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

    (void)VehicleHealthProcess();
    (void)BSP_ResetRecoveryObserveControl(HAL_GetTick(), RobotSafetyIsReady(),
                                          TaskMonitorAllCoreHealthyNow());

}
