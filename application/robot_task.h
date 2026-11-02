/** @file robot_task.h
* @brief 任务创建及常驻任务实现，仅由 robot.c 包含；重复包含到其他编译单元会重复定义。
 */
/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "main.h"
#include "platform_os.h"

#include "robot.h"
#include "robot_task_create.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "ins_task.h"
#include "motor_task.h"
#if MC02_SELECTED_DRIVER_DJI
#include "dji_motor.h"
#endif
#if MC02_FEATURE_REFEREE || MC02_FEATURE_UI
#include "referee_task.h"
#endif
#include "daemon.h"
#if MC02_SELECTED_DRIVER_HT
#include "HT04.h"
#endif
#if MC02_FEATURE_ALARM
#include "robot_alarm.h"
#endif
#include "task_monitor.h"
#if MC02_FEATURE_LCD
#include "lcd_task.h"
#endif
#include "bsp_crash.h"
#include "bsp_safety.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#endif
#include "init_diagnostic.h"
#include "robot_safety.h"

#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_usart.h"
#include "bsp_can.h"

#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_ROBOT)
/* 保留完整任务轮次最大耗时，供固定只读快照与任务栈数据一起核对。 */
volatile DWT_Probe_t r25_task_ins = {0};
volatile DWT_Probe_t r25_task_motor = {0};
volatile DWT_Probe_t r25_task_daemon = {0};
volatile DWT_Probe_t r25_task_robot = {0};
#endif


/** @brief 初始化 INS 后约每 1 ms 处理采样和姿态；失败时只维持诊断循环。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartINSTASK(void const *argument);
/** @brief 约每 1 ms 派发通信；仅机器人配置会继续运行电机计算和发送。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartMOTORTASK(void const *argument);
/** @brief 约每 10 ms 检查设备和任务，全部正常时才喂狗。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartDAEMONTASK(void const *argument);
/** @brief 约每 5 ms 运行整车控制和诊断。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartROBOTTASK(void const *argument);
#if MC02_FEATURE_UI
/** @brief 等待新鲜裁判机器人编号并周期运行 UI；每次循环至少等待 1 ms。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartUITASK(void const *argument);
#endif

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static volatile float ins_start;
    TaskMonitorFeed(TASK_MONITOR_INS);
    if (INS_Init() == NULL)
    {
#if defined(MC02_CONFIG_GENERATED)
        const uint8_t core_failure = RobotBuildMarkFailure(ROBOT_MODULE_IMU);
        if (core_failure != 0u)
        {
            InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                        INIT_FAILURE_RESOURCE, 0u);
            RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
            RobotSafetyRequestDisable();
            BSP_SafetyStartupLatchOutputsOff();
        }
        else
            InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                         INIT_FAILURE_RESOURCE, 0u);
#else
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                    INIT_FAILURE_RESOURCE, 0u);
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        RobotSafetyRequestDisable();
#if MC02_FEATURE_HEATER
        IMUHeaterForceOff();
#endif
        BSP_SafetyStartupLatchOutputsOff();
#endif
        // 算法初始化失败后保持输出关闭，但继续更新诊断和本任务记录，供人工读取。
        for (;;)
        {
            InitDiagnosticHeartbeat();
            TaskMonitorFeed(TASK_MONITOR_INS);
            PlatformTaskDelayMs(100u);
        }
    }
#if MC02_FEATURE_HEATER
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_HEATER) && !IMUHeaterInitSucceeded())
#else
    if (!IMUHeaterInitSucceeded())
#endif
    {
        IMUHeaterForceOff(); // 不论影响范围，先保持加热输出为零。
        if (RobotReportHeaterInitFailure())
        {
            RobotFaultReport(ROBOT_FAULT_INIT_FAILURE); // 记录影响核心的启动失败。
            RobotSafetyRequestDisable(); // 撤销整车运行许可。
            BSP_SafetyStartupLatchOutputsOff(); // 使用板级直接关断路径锁存输出关闭。
        }
    }
#endif
    InitDiagnosticMarkReady();
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_01);
    for (;;)
    {
        // 1kHz
        if (TaskMonitorBeginWork(TASK_MONITOR_INS) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(1u);
            continue;
        }
        /* 用轻量周期计数代替 DWT_GetTimeline_ms(): 后者内部有 64bit 除法, 1kHz 下也不便宜 */
        uint32_t ins_probe = DWT_ProbeStart();
        INS_Task();
        DWT_ProbeDone(&r25_task_ins, ins_probe);
        if (r25_task_ins.last_us > 1000u)
        {
            ins_start = DWT_GetTimeline_ms(); /* 便于 Live Watch 查看最近一次超时刻 */
            (void)ins_start;
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_02,
                            BSP_LOG_FIELD_U32(r25_task_ins.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_INS);
        // 视觉上行数据由 INS_Task 内的 VisionUpdateTx() 填充,
        // VisionSend() 统一在 RobotCMDTask(200Hz) 中调用, 避免多任务并发 DMA 发送
        (void)TaskMonitorSetWaiting(TASK_MONITOR_INS, TASK_MONITOR_WAIT_PERIOD);
        PlatformTaskDelayMs(1);
    }
}

#if MC02_FEATURE_COMM_DISPATCH
__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_03);
    for (;;)
    {
        if (TaskMonitorBeginWork(TASK_MONITOR_MOTOR) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(1u);
            continue;
        }
        uint32_t motor_probe = DWT_ProbeStart();
        MotorControlTask();
        DWT_ProbeDone(&r25_task_motor, motor_probe);
        if (r25_task_motor.last_us > 1000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_04,
                            BSP_LOG_FIELD_U32(r25_task_motor.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_MOTOR);
        (void)TaskMonitorSetWaiting(TASK_MONITOR_MOTOR, TASK_MONITOR_WAIT_PERIOD);
        PlatformTaskDelayMs(1);
    }
}
#endif

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_05);
#if LOG_TEST
    /* 类型化日志自测默认关闭；普通事件也需在bench_safe显式开启诊断输出。 */
    LogSelfTest();
#endif
#if CAN_SELFTEST_LOOPBACK
    /* FDCAN 回环自测(默认关闭): 必须在所有设备注册完成之后跑(否则 CANRegister 会重新
       配置总线把自测配置覆盖掉), 放在任务上下文里也便于用 HAL_GetTick 做超时 */
    CANRunLoopbackSelfTest();
#endif
#if MC02_BENCH_CRASH_DIAGNOSTICS
    /* 专用固件启动稳定后才检查许可；许可本身须由匹配Build ID的工具写入。 */
    static uint32_t s_crash_test_wait = 0u;
#endif
#if CAN_DJI_INJECT_TEST >= 1
    /* DJI 假反馈注入自测(默认关闭): 验证"中断入队 → CANProcessRx 派发 → 解码 → 多圈累加" */
    (void)CANRunDjiFeedbackInjectTest();
#endif
    for (;;)
    {
        // 100Hz
        if (TaskMonitorBeginWork(TASK_MONITOR_DAEMON) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(10u);
            continue;
        }
        uint32_t daemon_probe = DWT_ProbeStart();
#if LOG_TEST
        static uint32_t s_log_test_tick = 0u;
        if (++s_log_test_tick >= 200u)
        {
            s_log_test_tick = 0u;
            LogSelfTest();
        }
#endif
        // 约每 10 ms 停止出错或到期的异步 SPI，停止失败的总线保持隔离。
        (void)SPIProcessTimeouts();
        // 电压诊断只记录读取结果，内部每 250ms 更新一次，不参与控制许可。
        BSP_ADCUpdateDiagnostics();
        DaemonTask();
#if MC02_FEATURE_ALARM
#if defined(MC02_CONFIG_GENERATED)
        // 启动计划未接受告警模块或其依赖后续失败时，周期任务停止访问告警状态。
        if (RobotBuildModuleStarted(ROBOT_MODULE_ALARM))
#endif
            (void)RobotAlarmTask();
#endif
        CANHealthMonitor(); /* 10Hz: 检测 BusOff 并恢复(空总线/异常时防止 CAN 永久死掉) */
        TaskMonitorTick();
#if MC02_BENCH_CRASH_DIAGNOSTICS
        // 给其他任务留出启动时间；无许可时检查无副作用，有许可时只消费一次后复位。
        if (s_crash_test_wait < 100u)
            ++s_crash_test_wait;
        else
            CrashLogSelfTest();
#endif
#if CAN_DJI_INJECT_TEST >= 2
        DJIInjectStressTick(); /* 压测模式: 持续往接收队列里灌帧 */
#endif
        DWT_ProbeDone(&r25_task_daemon, daemon_probe);
        if (r25_task_daemon.last_us > 10000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_06,
                            BSP_LOG_FIELD_U32(r25_task_daemon.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_DAEMON);
        (void)TaskMonitorSetWaiting(TASK_MONITOR_DAEMON, TASK_MONITOR_WAIT_PERIOD);
        PlatformTaskDelayMs(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_07);
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        if (TaskMonitorBeginWork(TASK_MONITOR_ROBOT) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(5u);
            continue;
        }
        uint32_t robot_probe = DWT_ProbeStart();
        RobotTask();
        DWT_ProbeDone(&r25_task_robot, robot_probe);
        if (r25_task_robot.last_us > 5000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_08,
                            BSP_LOG_FIELD_U32(r25_task_robot.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_ROBOT);
        (void)TaskMonitorSetWaiting(TASK_MONITOR_ROBOT, TASK_MONITOR_WAIT_PERIOD);
        PlatformTaskDelayMs(5);
    }
}

#if MC02_FEATURE_UI
__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    (void)argument;
    // 只登记一次请求；每轮最多提交一帧，身份失效后由显式MyUIInit恢复。
    (void)MyUIInit();
    for (;;)
    {
        (void)UITask();
        PlatformTaskDelayMs(1u);
    }
}
#endif
