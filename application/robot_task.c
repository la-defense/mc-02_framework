/** @file robot_task.c
 * @brief 运行已放行的常驻任务，并在禁热配置中使用固定拍点。
 */
#include "robot_task.h"
#include "platform_os.h"
#include "robot_monitor_plan.h"
#if MC02_TASK_CHASSIS
#include "robot_control_tasks.h"
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

typedef struct {
    TaskMonitor_Id_e monitor; // 本任务在看门狗监控中的编号。
    uint32_t delay_ms; // 旧配置的轮间延迟，或错误后仅用于让出处理器。
    uint32_t budget_us; // 现有 DWT 耗时告警的上限。
    void (*clock_fault)(void); // 控制任务可在时钟失效时立即撤销自己的输出。
    PlatformPeriodicClock_t clock; // 任务私有的下一拍，计数从运行放行后开始。
    PlatformPeriodicRelease_t release; // 等待成功取得的拍点，用于完成检查。
    uint8_t fixed; // 1 使用固定拍点，0 保留轮间相对延迟。
    uint8_t ready; // 时钟初始化或等待失败后保持为 0。
} RobotTaskCycle_t;

// 锁定时钟故障，不继续工作或报告新的任务进度。
static void StopTaskClock(RobotTaskCycle_t *cycle)
{
    cycle->ready = 0u; // 后续轮次不会重新领取拍点。
    if (cycle->clock_fault != NULL) cycle->clock_fault(); // 控制任务先撤销自己的输出。
}

// 依赖启动协调者先发布监控配置并放行，再建立任务自己的拍点。
static RobotTaskCycle_t OpenConfiguredTaskCycle(const PlatformPeriodicConfig_t *config,
    TaskMonitor_Id_e monitor, uint8_t fixed, void (*clock_fault)(void))
{
    while (!TaskMonitorRuntimeReleased())
        PlatformTaskDelayMs(1u); // 等待期间不执行任务业务，也不报告一轮工作完成。
    RobotTaskCycle_t cycle = {
        .monitor = monitor, // 后续进度统一报告给此监控项。
        .delay_ms = config->period_us / 1000u, // 现有任务均使用整毫秒的名义间隔。
        .budget_us = config->budget_us, // 耗时告警复用启动时检查的配置。
        .clock_fault = clock_fault, // 记录本任务的计时故障处理。
        .fixed = fixed, // 本次启动后不改变等待方式。
    };
    if (cycle.fixed)
    {
        cycle.ready = (uint8_t)(PlatformTaskPeriodicInit(&cycle.clock, config) == PLATFORM_PERIODIC_READY);
        if (!cycle.ready) StopTaskClock(&cycle); // 初始化失败也不能留下有效的轮组目标。
    }
    return cycle;
}

// 保留既有常驻任务的配置选择；独立底盘单独指定固定拍点。
static RobotTaskCycle_t OpenTaskCycle(RobotPeriodicTask_t task, TaskMonitor_Id_e monitor)
{
#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
    const uint8_t fixed = 1u; // 禁热和观察版使用固定拍点。
#else
    const uint8_t fixed = 0u; // 未独立迁移的正式任务继续相对等待。
#endif
    return OpenConfiguredTaskCycle(RobotGetTaskPeriod(task), monitor, fixed, NULL);
}

// 固定周期任务先领取拍点，再向监控请求执行；失败时让出处理器。
static uint8_t BeginTaskCycle(RobotTaskCycle_t *cycle)
{
    if (cycle->fixed)
    {
        if (cycle->ready && PlatformTaskPeriodicWait(&cycle->clock, &cycle->release) != PLATFORM_PERIODIC_READY)
            StopTaskClock(cycle); // 只在首次失败时调用输出撤销。
        if (!cycle->ready)
        {
            PlatformTaskDelayMs(cycle->delay_ms); // 仅让出处理器，监控会观察到工作停止。
            return 0u;
        }
    }
    if (TaskMonitorBeginWork(cycle->monitor) == TASK_MONITOR_RESULT_OK) return 1u; // 监控允许才执行工作。
    if (!cycle->fixed) PlatformTaskDelayMs(cycle->delay_ms); // 相对等待的任务按原间隔再次请求许可。
    return 0u;
}

// 检查完成时刻，再交付任务进度。
static void CompleteTaskCycle(RobotTaskCycle_t *cycle)
{
    uint8_t yield_for_peers = 0u; // 仅在下一拍已经到达时额外让出处理器。
    if (cycle->fixed)
    {
        PlatformPeriodicTiming_t timing; // 完成检查使用与等待相同的时钟。
        if (PlatformTaskPeriodicFinish(&cycle->clock, &cycle->release, &timing) != PLATFORM_PERIODIC_READY)
        {
            StopTaskClock(cycle); // 时钟不可信时撤销本任务输出并停止报告进度。
            return;
        }
        const uint32_t start_lag = cycle->release.observed_tick - cycle->release.scheduled_tick; // 开始前已过的刻度。
        yield_for_peers = (uint8_t)(start_lag + timing.work_ticks >= cycle->clock.period_ticks); // 下一拍是否已到。
    }
    (void)TaskMonitorCompleteWork(cycle->monitor); // 记录完成时间。
    (void)TaskMonitorSetWaiting(cycle->monitor, TASK_MONITOR_WAIT_PERIOD); // 标记接下来等待下一拍。
    if (yield_for_peers)
        PlatformTaskDelayMs(1u); // 下一拍已到，先让出一个 RTOS 刻度，之后仍按原相位取最近一拍。
    else if (!cycle->fixed)
        PlatformTaskDelayMs(cycle->delay_ms); // 旧配置沿用轮间相对延迟。
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    (void)argument;
    static volatile float ins_start;
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_01); // 记录姿态任务进入周期循环。
    RobotTaskCycle_t cycle = OpenTaskCycle(ROBOT_PERIODIC_INS, TASK_MONITOR_INS);
    for (;;)
    {
        if (!BeginTaskCycle(&cycle)) continue; // 只有领取拍点且监控接受才采样。
        // 使用轻量周期计数，避免每轮读取带 64 位除法的时间线。
        uint32_t ins_probe = PlatformProbeStart(); // 记录本轮计算起点。
        INS_Task(); // 读取样本并执行姿态更新。
        PlatformProbeDone(&r25_task_ins, ins_probe); // 保存本轮计算耗时。
        if (r25_task_ins.last_us > cycle.budget_us)
        {
            ins_start = PlatformProbeTimelineMs(); // 保存最近一次超时的时间供调试查看。
            (void)ins_start; // 该变量仅供调试器读取。
            // 记录超过 1 ms 的实际耗时。
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_02,
                            BSP_LOG_FIELD_U32(r25_task_ins.last_us));
        }
        // 视觉上行数据由 INS_Task 内的 VisionUpdateTx() 填充,
        // VisionSend() 统一在 RobotCMDTask(200Hz) 中调用, 避免多任务并发 DMA 发送
        CompleteTaskCycle(&cycle);
    }
}

#if MC02_FEATURE_COMM_DISPATCH
__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    (void)argument;
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_03);
    RobotTaskCycle_t cycle = OpenTaskCycle(ROBOT_PERIODIC_COMM, TASK_MONITOR_MOTOR);
    for (;;)
    {
        if (!BeginTaskCycle(&cycle)) continue; // 时钟或监控失败时不运行本轮派发。
        uint32_t motor_probe = PlatformProbeStart();
        MotorControlTask();
        PlatformProbeDone(&r25_task_motor, motor_probe);
        if (r25_task_motor.last_us > cycle.budget_us)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_04,
                            BSP_LOG_FIELD_U32(r25_task_motor.last_us));
        }
        CompleteTaskCycle(&cycle);
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
    (void)argument;
    RobotTaskCycle_t cycle = OpenTaskCycle(ROBOT_PERIODIC_DAEMON, TASK_MONITOR_DAEMON);
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
        if (!BeginTaskCycle(&cycle)) continue; // 监控及维护都在已允许的任务拍点执行。
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
        if (r25_task_daemon.last_us > cycle.budget_us)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_06,
                            BSP_LOG_FIELD_U32(r25_task_daemon.last_us));
        }
        CompleteTaskCycle(&cycle);
    }
}

#if MC02_BENCH_TIM5_DIAGNOSTICS
volatile uint32_t mc02_tim5_probe_result = UINT32_MAX; // 调试器读取，UINT32_MAX 表示尚未完成。
volatile uint32_t mc02_tim5_probe_releases; // 一次性检查中实际领取的拍数。
volatile uint32_t mc02_tim5_probe_skipped; // 同时记录由于唤醒迟到而放弃的拍数。

static void RunTim5Probe(void)
{
    const PlatformPeriodicConfig_t *config = RobotGetTaskPeriod(ROBOT_PERIODIC_TIM5_PROBE); // 复用启动时检查过的配置。
    PlatformPeriodicClock_t clock;
    PlatformPeriodicResult_t result = PlatformTaskHighResOpen(&clock, config); // 运行时再次核对并取得一个独立通道。
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
    (void)argument;
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_07);
#if MC02_BENCH_TIM5_DIAGNOSTICS
    while (!TaskMonitorRuntimeReleased())
        PlatformTaskDelayMs(1u); // 自测本身也要等待统一放行。
    RunTim5Probe(); // 执行一次有限的硬件时序检查，随后恢复普通裸板诊断。
#endif
    RobotTaskCycle_t cycle = OpenTaskCycle(ROBOT_PERIODIC_DIAGNOSTIC, TASK_MONITOR_ROBOT);
    for (;;)
    {
        if (!BeginTaskCycle(&cycle)) continue; // 放行、拍点与监控许可全部通过才执行。
        uint32_t robot_probe = PlatformProbeStart();
        RobotTask();
        PlatformProbeDone(&r25_task_robot, robot_probe);
        if (r25_task_robot.last_us > cycle.budget_us)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_08,
                            BSP_LOG_FIELD_U32(r25_task_robot.last_us));
        }
        CompleteTaskCycle(&cycle);
    }
}

#if MC02_TASK_CHASSIS
__attribute__((noreturn)) void StartCHASSISTASK(void const *argument)
{
    (void)argument;
    const RobotChassisTaskConfig_t *config = RobotGetChassisTaskConfig(); // 创建者已检查统一监控配置。
    if (config == NULL)
        for (;;) PlatformTaskDelayMs(1u); // 启动契约未满足时不访问设备，也不代报进度。
    RobotTaskCycle_t cycle = OpenConfiguredTaskCycle(&config->timing, TASK_MONITOR_CHASSIS,
                                                    1u, ChassisStopAndClear); // 放行后使用底盘私有时钟。
    for (;;)
    {
        if (!RobotBuildModuleStarted(ROBOT_MODULE_CHASSIS))
        {
            ChassisStopAndClear(); // 运行期间的依赖失效永久停止本任务工作。
            for (;;) PlatformTaskDelayMs(cycle.delay_ms);
        }
        if (!BeginTaskCycle(&cycle)) continue; // 本轮需要有效拍点和监控许可。
        ChassisTask(); // 独立任务是本构建中唯一的底盘命令消费者。
        CompleteTaskCycle(&cycle); // 完成时钟有效后才提交监控进度。
    }
}
#endif

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
