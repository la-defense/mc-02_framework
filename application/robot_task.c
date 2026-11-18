/** @file robot_task.c
 * @brief 实现现有常驻任务；周期、调度和故障处理保持原有行为。
 */
#include "robot_task.h"
#include "platform_os.h"
#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
#include "platform_periodic.h"
#endif
#include <stddef.h>

#include "robot.h"
#include "robot_startup.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "ins_task.h"
#include "motor_task.h"
#if MC02_SELECTED_DRIVER_DJI
#include "DJImotor/dji_motor.h"
#endif
#if MC02_FEATURE_REFEREE || MC02_FEATURE_UI
#include "referee_task.h"
#endif
#include "daemon.h"
#if MC02_FEATURE_ALARM
#include "robot_alarm.h"
#endif
#include "task_monitor.h"
#include "bsp_crash.h"
#include "init_diagnostic.h"

#include "bsp_log.h"
#include "platform_probe.h"
#include "platform_maintenance.h"
#include "platform_can_diagnostic.h"

#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_ROBOT)
/* 保留完整任务轮次最大耗时，供固定只读快照与任务栈数据一起核对。 */
volatile PlatformProbe_t r25_task_ins = {0};
volatile PlatformProbe_t r25_task_motor = {0};
volatile PlatformProbe_t r25_task_daemon = {0};
volatile PlatformProbe_t r25_task_robot = {0};
#endif


__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static volatile float ins_start;
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_01); // 记录姿态任务进入周期循环。
    for (;;)
    {
        // 1kHz
        if (TaskMonitorBeginWork(TASK_MONITOR_INS) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(1u); // 本轮不能开始时先让出处理器。
            continue; // 下一周期重新请求工作许可。
        }
        // 使用轻量周期计数，避免每轮读取带 64 位除法的时间线。
        uint32_t ins_probe = PlatformProbeStart(); // 记录本轮计算起点。
        INS_Task(); // 读取样本并执行姿态更新。
        PlatformProbeDone(&r25_task_ins, ins_probe); // 保存本轮计算耗时。
        if (r25_task_ins.last_us > 1000u)
        {
            ins_start = PlatformProbeTimelineMs(); // 保存最近一次超时的时间供调试查看。
            (void)ins_start; // 该变量仅供调试器读取。
            // 记录超过 1 ms 的实际耗时。
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_02,
                            BSP_LOG_FIELD_U32(r25_task_ins.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_INS); // 结束本轮进度记录。
        // 视觉上行数据由 INS_Task 内的 VisionUpdateTx() 填充,
        // VisionSend() 统一在 RobotCMDTask(200Hz) 中调用, 避免多任务并发 DMA 发送
        (void)TaskMonitorSetWaiting(TASK_MONITOR_INS, TASK_MONITOR_WAIT_PERIOD); // 标记进入周期等待。
        PlatformTaskDelayMs(1); // 等待下一轮任务执行。
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
        uint32_t motor_probe = PlatformProbeStart();
        MotorControlTask();
        PlatformProbeDone(&r25_task_motor, motor_probe);
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

void RobotRunStartupDiagnostics(void)
{
#if CAN_SELFTEST_LOOPBACK
    RobotStartupRecordCanDiagnostic(PlatformCanRunLoopbackDiagnostic()); // 报告诊断结果；核心隔离由后续检查处理。
#endif
#if CAN_DJI_INJECT_TEST >= 1
    (void)CANRunDjiFeedbackInjectTest(); // 诊断自行推进收发，避免与电机任务并发。
#endif
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    while (!TaskMonitorRuntimeReleased())
        PlatformTaskDelayMs(1u); // 等待协调者放行，再执行诊断和设备访问。
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_05);
#if LOG_TEST
    /* 类型化日志自测默认关闭；普通事件也需在bench_safe显式开启诊断输出。 */
    LogSelfTest();
#endif
#if MC02_BENCH_CRASH_DIAGNOSTICS
    /* 专用固件启动稳定后才检查许可；许可本身须由匹配Build ID的工具写入。 */
    static uint32_t s_crash_test_wait = 0u;
#endif
    for (;;)
    {
        // 100Hz
        if (TaskMonitorBeginWork(TASK_MONITOR_DAEMON) != TASK_MONITOR_RESULT_OK)
        {
            PlatformTaskDelayMs(10u);
            continue;
        }
        uint32_t daemon_probe = PlatformProbeStart();
#if LOG_TEST
        static uint32_t s_log_test_tick = 0u;
        if (++s_log_test_tick >= 200u)
        {
            s_log_test_tick = 0u;
            LogSelfTest();
        }
#endif
        // 约每 10 ms 停止出错或到期的异步 SPI，停止失败的总线保持隔离。
        PlatformMaintainSpi();
        // 电压诊断只记录读取结果，内部每 250ms 更新一次，不参与控制许可。
        PlatformMaintainAnalog();
        DaemonTask();
#if MC02_FEATURE_ALARM
#if defined(MC02_CONFIG_GENERATED)
        // 启动计划未接受告警模块或其依赖后续失败时，周期任务停止访问告警状态。
        if (RobotBuildModuleStarted(ROBOT_MODULE_ALARM))
#endif
            (void)RobotAlarmTask();
#endif
        PlatformMaintainCan(); /* 10Hz: 检查总线故障并按现有规则恢复。 */
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
        PlatformProbeDone(&r25_task_daemon, daemon_probe);
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

#if MC02_BENCH_TIM5_DIAGNOSTICS
volatile uint32_t mc02_tim5_probe_result = UINT32_MAX; // 调试器读取，UINT32_MAX 表示尚未完成。
volatile uint32_t mc02_tim5_probe_releases; // 一次性检查中实际领取的拍数。
volatile uint32_t mc02_tim5_probe_skipped; // 同时记录由于唤醒迟到而放弃的拍数。

static void RunTim5Probe(void)
{
    const PlatformPeriodicConfig_t config = {
        .period_us = 500u, // 裸板测试拍点，单位 μs。
        .budget_us = 500u, // 自测只处理计数，不启动业务任务。
    };
    PlatformPeriodicClock_t clock;
    PlatformPeriodicResult_t result = PlatformTaskHighResOpen(&clock, &config); // 先确认硬件可独占。
    if (result == PLATFORM_PERIODIC_READY)
    {
        for (unsigned i = 0u; i < 8u; ++i)
        {
            PlatformPeriodicRelease_t release;
            result = PlatformTaskHighResWait(&clock, &release); // 每轮有限等待一个拍点。
            if (result != PLATFORM_PERIODIC_READY) break; // 首次失效后结束检查。
            ++mc02_tim5_probe_releases; // 诊断不运行任何业务控制。
            mc02_tim5_probe_skipped += release.skipped; // 迟到可由调试器观察。
        }
        const PlatformPeriodicResult_t close = PlatformTaskHighResClose(&clock); // 成功或超时都释放定时器。
        if (result == PLATFORM_PERIODIC_READY) result = close; // 保留先发生的错误。
    }
    mc02_tim5_probe_result = (uint32_t)result; // 释放完成后发布最终状态。
}
#endif

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_07);
#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
    while (!TaskMonitorRuntimeReleased())
        PlatformTaskDelayMs(1u); // 诊断周期从启动完成后开始计时。
#if MC02_BENCH_TIM5_DIAGNOSTICS
    RunTim5Probe(); // 执行一次有限的硬件时序检查，随后恢复普通裸板诊断。
#endif
    const PlatformPeriodicConfig_t config = {
        .period_us = 5000u, // 保留裸板整车诊断的名义周期 5 ms。
        .budget_us = 5000u, // 对应现有单轮耗时告警阈值。
    };
    PlatformPeriodicClock_t clock;
    uint8_t periodic_ready = (uint8_t)(PlatformTaskPeriodicInit(&clock, &config) == PLATFORM_PERIODIC_READY); // 只接受后端能准确表示的周期。
#endif
    for (;;)
    {
#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
        PlatformPeriodicRelease_t release; // 每次只领取最近到期的一拍。
        if (!periodic_ready || PlatformTaskPeriodicWait(&clock, &release) != PLATFORM_PERIODIC_READY)
        {
            periodic_ready = 0u; // 后端错误后停止工作报告，由任务监控按既有规则处理。
            PlatformTaskDelayMs(5u); // 错误分支仅让出处理器，不继续执行诊断周期。
            continue;
        }
#endif
        if (TaskMonitorBeginWork(TASK_MONITOR_ROBOT) != TASK_MONITOR_RESULT_OK)
        {
#if !defined(MC02_PROFILE_BENCH_SAFE) && !defined(MC02_PROFILE_COMMISSIONING)
            PlatformTaskDelayMs(5u);
#endif
            continue;
        }
        uint32_t robot_probe = PlatformProbeStart();
        RobotTask();
        PlatformProbeDone(&r25_task_robot, robot_probe);
        if (r25_task_robot.last_us > 5000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_08,
                            BSP_LOG_FIELD_U32(r25_task_robot.last_us));
        }
        (void)TaskMonitorCompleteWork(TASK_MONITOR_ROBOT);
        (void)TaskMonitorSetWaiting(TASK_MONITOR_ROBOT, TASK_MONITOR_WAIT_PERIOD);
#if !defined(MC02_PROFILE_BENCH_SAFE) && !defined(MC02_PROFILE_COMMISSIONING)
        PlatformTaskDelayMs(5);
#endif
    }
}

#if MC02_FEATURE_UI
__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    (void)argument;
    while (!TaskMonitorRuntimeReleased())
        PlatformTaskDelayMs(1u); // 启动失败时不提交裁判界面消息。
    // 只登记一次请求；每轮最多提交一帧，身份失效后由显式MyUIInit恢复。
    (void)MyUIInit();
    for (;;)
    {
        (void)UITask();
        PlatformTaskDelayMs(1u);
    }
}
#endif
