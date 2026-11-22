/** @file motor_task.c
* @brief 统一接收派发顺序和电机周期计算，提供各段耗时记录。
*/
#include "motor_task.h"
#include "motor_pid.h"
#include "../../bsp/dwt/platform_probe.h"
#include "../../bsp/os/platform_dispatch.h"
#include "../../application/config/build_selection.h"
#if MC02_SELECTED_DRIVER_LK
#include "LKmotor/lk_motor_task.h"
#endif
#if MC02_SELECTED_DRIVER_DJI
#include "DJImotor/dji_motor_task.h"
#endif
#include "usb_transport.h"
#include "daemon.h"
#include <stddef.h>

/* ---------------- 性能探针(2026-09, CPU 占用排查) ---------------- */
volatile PlatformProbe_t motor_prof_all = {0};
volatile PlatformProbe_t motor_prof_dji = {0};
volatile PlatformProbe_t motor_prof_lk = {0};

#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
#include "../../application/topics/robot_can_bridge_task.h"
#if defined(MC02_CONFIG_GENERATED)
#include "../../application/config/vehicle_selection.h"
#endif
#endif

PlatformPeriodicResult_t MotorTaskScheduleInit(MotorTaskSchedule_t *schedule, const MotorTaskPeriods_t *periods,
    uint32_t dispatch_us, uint32_t quantum_us, uint32_t first_tick)
{
    if (schedule == NULL || periods == NULL || dispatch_us == 0u ||
        periods->compute_us < dispatch_us || periods->transmit_us < dispatch_us ||
        periods->compute_us > MOTOR_PID_DT_MAX_US ||
        periods->compute_us % dispatch_us != 0u || periods->transmit_us % dispatch_us != 0u ||
        periods->transmit_us > MOTOR_DJI_QUEUE_LIFETIME_MS * 1000u)
        return PLATFORM_PERIODIC_CONFIG; // 两个周期须能由轮询表示，并满足 PID 计时及队列期限。

    const PlatformPeriodicConfig_t compute = {
        .period_us = periods->compute_us, // 计算单独配置周期。
        .budget_us = dispatch_us, // 两项都在一轮轮询内推进，轮询预算另由任务监控。
    };
    const PlatformPeriodicConfig_t transmit = {
        .period_us = periods->transmit_us, // 发送单独配置周期。
        .budget_us = dispatch_us, // 仅用于阶段拍点，完成时限按整轮测量。
    };
    MotorTaskSchedule_t candidate; // 两个拍点均有效后才交付。
    if (PlatformPeriodicInit(&candidate.compute, &compute, quantum_us, first_tick) != PLATFORM_PERIODIC_READY ||
        PlatformPeriodicInit(&candidate.transmit, &transmit, quantum_us, first_tick) != PLATFORM_PERIODIC_READY)
        return PLATFORM_PERIODIC_CONFIG; // 平台刻度必须精确表示已选周期和轮询间隔。
    *schedule = candidate;
    return PLATFORM_PERIODIC_READY;
}

static void MotorTaskStep(uint8_t compute_due, uint8_t transmit_due,
                          MotorTaskTransmitCheck_t check_transmit, void *context)
{
    uint32_t probe_all = PlatformProbeStart();

    // 先按顺序解码已排队的反馈，再使用这些测量值计算；不要另建并发派发调用者。
    PlatformDispatchCanRx();
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
    // 收发和同步检查由同一任务串行执行，避免接收状态被并发改写。
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_COMMUNICATION)) // 辅助通信降级后不再运行桥接。
#endif
        RobotCanBridgeService();
#endif
    // UART 和 USB 也在这里运行模块解析，避免将协议计算放在中断里；排队过时的数据由 BSP 跳过。
    PlatformDispatchUartRx();
#if !defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    (void)USBTransportProcessRx();
#endif

    // 裁判 UART 已完成本轮解析后，再由电机任务处理有界离线恢复事件。
    (void)DaemonProcessTaskEvents(DAEMON_CONSUMER_MOTOR_TASK, DAEMON_EVENT_MAX_DEPTH, NULL);

#if !defined(MC02_PROFILE_BENCH_SAFE)
    if (compute_due)
    {
        uint32_t probe_seg = PlatformProbeStart(); // 只在计算到期时记录各驱动耗时。
#if MC02_SELECTED_DRIVER_DJI
        DJIMotorControl();
#endif
        PlatformProbeDone(&motor_prof_dji, probe_seg);
        probe_seg = PlatformProbeStart();
#if MC02_SELECTED_DRIVER_LK
        LKMotorControl();
#endif
        PlatformProbeDone(&motor_prof_lk, probe_seg);
    }
#else
    (void)compute_due; // 裸板仅派发通信，不运行电机计算。
#endif

#if MC02_FEATURE_MOTOR_CONTROL
    if (transmit_due && (check_transmit == NULL || check_transmit(context)))
    {
        PlatformDispatchCanTx(); // 只在发送拍点推进按原期限和许可检查的队列。
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
#if defined(MC02_CONFIG_GENERATED)
        if (RobotBuildModuleStarted(ROBOT_MODULE_COMMUNICATION)) // 辅助通信降级后不发送桥接帧。
#endif
            RobotCanBridgeServiceTx();
#endif
    }
#else
    (void)transmit_due; // 该构建无 CAN 周期发送。
    (void)check_transmit; // 没有发送，也无需读取发送前时钟。
    (void)context;
#endif
    PlatformProbeDone(&motor_prof_all, probe_all);
}

PlatformPeriodicResult_t MotorTaskRunScheduled(MotorTaskSchedule_t *schedule, uint32_t now_tick,
    MotorTaskTransmitCheck_t check_transmit, void *context)
{
    if (schedule == NULL || check_transmit == NULL) return PLATFORM_PERIODIC_CONFIG; // 固定拍点必须提供发送前复查。
    MotorTaskSchedule_t candidate = *schedule; // 任一时钟失效都不推进原有拍点。
    PlatformPeriodicRelease_t compute_release;
    PlatformPeriodicRelease_t transmit_release;
    const PlatformPeriodicResult_t compute = PlatformPeriodicPoll(&candidate.compute, now_tick, &compute_release);
    if (compute != PLATFORM_PERIODIC_READY && compute != PLATFORM_PERIODIC_WAIT) return compute;
    const PlatformPeriodicResult_t transmit = PlatformPeriodicPoll(&candidate.transmit, now_tick, &transmit_release);
    if (transmit != PLATFORM_PERIODIC_READY && transmit != PLATFORM_PERIODIC_WAIT) return transmit;
    *schedule = candidate; // 两项都有效，提交本轮已领取的拍点。
    MotorTaskStep(compute == PLATFORM_PERIODIC_READY, transmit == PLATFORM_PERIODIC_READY,
                 check_transmit, context); // 同时到期时先计算，再检查当前时刻是否还可发送。
    return PLATFORM_PERIODIC_READY;
}

void MotorControlTask(void)
{
    MotorTaskStep(1u, 1u, NULL, NULL); // 旧调用者仍在同一轮处理计算和发送。
}
