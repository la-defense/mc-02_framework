/** @file robot_task_cycle_test.c
 * @brief 从实际任务入口验证放行、监控许可和唤醒故障后的行为。
 */
#include "robot_task.h"
#include "platform_os.h"
#include "platform_periodic.h"
#include "platform_probe.h"
#include "robot.h"
#include "task_monitor.h"
#include "test_check.h"
#if MC02_TASK_CONTROL
#include "robot_monitor_plan.h"
#include "config/vehicle_selection.h"
#endif
#if MC02_TASK_MOTOR_SPLIT || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
#include "robot_safety.h"
#endif
#include <setjmp.h>
#include <string.h>

static jmp_buf task_exit;
static TaskMonitor_Id_e expected_monitor;
static uint32_t expected_period_ms;
static unsigned release_delays, initializations, waits, begins, work, finishes, completes, idle_reports, fault_delays;
static unsigned fairness_delays;
static uint8_t hard_deadline; // 区分硬期限控制和普通常驻任务的完成策略。
static uint8_t finish_error; // 注入实际工作以后才发生的计数错误。
#if MC02_TASK_CONTROL || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
static unsigned stops, timing_faults;
static TaskMonitor_FaultReason_e expected_fault = TASK_MONITOR_FAULT_PERIOD_CLOCK;
#endif
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS) || MC02_TASK_CONTROL
static uint8_t skipped_release; // 注入等待成功但漏过了上一拍的结果。
static uint8_t deadline_error; // 注入工作完成时已经越过下一计划拍。
static uint8_t budget_error; // 注入执行时间已超过本轮预算。
#endif
#if MC02_TASK_CONTROL
static RobotControlTaskConfig_t control = {
    .timing = {.period_us = 4000u, .budget_us = 4000u}, // 主机试验依赖的控制拍点，单位 μs。
    .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 整毫秒场景固定使用 RTOS。
};
const RobotControlTaskConfig_t *RobotGetControlTaskConfig(RobotControlTaskId_t task)
{
    TEST_CHECK((task == ROBOT_CONTROL_TASK_CHASSIS && expected_monitor == TASK_MONITOR_CHASSIS) ||
        (task == ROBOT_CONTROL_TASK_GIMBAL && expected_monitor == TASK_MONITOR_GIMBAL) ||
        (task == ROBOT_CONTROL_TASK_SHOOTER && expected_monitor == TASK_MONITOR_SHOOTER));
    return &control;
}
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    TEST_CHECK(release_delays == 1u && ((module == ROBOT_MODULE_CHASSIS && expected_monitor == TASK_MONITOR_CHASSIS) ||
        (module == ROBOT_MODULE_GIMBAL && expected_monitor == TASK_MONITOR_GIMBAL) ||
        (module == ROBOT_MODULE_SHOOTER && expected_monitor == TASK_MONITOR_SHOOTER)));
    return 1u; // 设备接线和资源初始化由替身代替。
}
static void StopControl(TaskMonitor_Id_e monitor)
{
    TEST_CHECK(expected_monitor == monitor && work == 1u);
    TEST_CHECK(finish_error ? (finishes == 1u && completes == 0u) : (waits == 3u && completes == 1u));
    ++stops; // 时钟故障只撤销一次，下一轮仍禁止工作。
}
#endif
#if MC02_TASK_CHASSIS
void ChassisStopAndClear(void) { StopControl(TASK_MONITOR_CHASSIS); }
#endif
#if MC02_TASK_GIMBAL
static unsigned gimbal_feedback;
void GimbalStopAndClear(void) { StopControl(TASK_MONITOR_GIMBAL); }
#endif
#if MC02_TASK_SHOOTER
void ShootStopAndClear(void) { StopControl(TASK_MONITOR_SHOOTER); }
#endif

#if MC02_TASK_MOTOR_SPLIT
static uint8_t before_send_budget; // 本轮接收和计算已耗尽发送前的执行预算。
static uint8_t before_send_deadline; // 开始较迟，发送前刚到绝对截止点。
static unsigned motor_sends;

const PlatformPeriodicConfig_t *RobotGetTaskPeriod(RobotPeriodicTask_t id)
{
    static const PlatformPeriodicConfig_t dispatch = {1000u, 1000u}; // 测试沿用电机任务的 RTOS 派发拍点。
    static const PlatformPeriodicConfig_t late_dispatch = {2000u, 2000u}; // 留出一个刻度模拟调度延迟。
    static const PlatformPeriodicConfig_t ins = {1000u, 1000u}; // 姿态任务名义周期和预算。
    static const PlatformPeriodicConfig_t robot = {5000u, 5000u}; // 整车任务名义周期和预算。
    static const PlatformPeriodicConfig_t daemon = {10000u, 10000u}; // 设备维护任务名义周期和预算。
    if (id == ROBOT_PERIODIC_INS && expected_monitor == TASK_MONITOR_INS) return &ins;
    if (id == ROBOT_PERIODIC_DIAGNOSTIC && expected_monitor == TASK_MONITOR_ROBOT) return &robot;
    if (id == ROBOT_PERIODIC_DAEMON && expected_monitor == TASK_MONITOR_DAEMON) return &daemon;
    TEST_CHECK(id == ROBOT_PERIODIC_COMM && expected_monitor == TASK_MONITOR_MOTOR);
    return before_send_deadline ? &late_dispatch : &dispatch; // 绝对截止点场景需要正的开始延迟。
}

const MotorTaskPeriods_t *RobotGetMotorTaskPeriods(void)
{
    static const MotorTaskPeriods_t periods = {2000u, 4000u}; // 两个周期均与测试轮询间隔对齐。
    return &periods;
}

PlatformPeriodicResult_t MotorTaskScheduleInit(MotorTaskSchedule_t *schedule, const MotorTaskPeriods_t *periods,
    uint32_t dispatch_us, uint32_t quantum_us, uint32_t first_tick)
{
    TEST_CHECK(schedule != NULL && periods == RobotGetMotorTaskPeriods() && first_tick == 0u);
    TEST_CHECK(dispatch_us == expected_period_ms * 1000u && quantum_us == 1000u && initializations == 1u);
    return PLATFORM_PERIODIC_READY; // 电机拍点计算由独立的生产测试验证。
}

PlatformPeriodicResult_t MotorTaskRunScheduled(MotorTaskSchedule_t *schedule, uint32_t now_tick,
    MotorTaskTransmitCheck_t check_transmit, void *context)
{
    TEST_CHECK(schedule != NULL && now_tick == before_send_deadline && work == 0u && waits == 2u);
    TEST_CHECK(check_transmit != NULL && context != NULL);
    ++work; // 模拟已执行接收和闭环计算。
    if (check_transmit(context)) ++motor_sends; // 记录通过检查后才进入发送的次数。
    return PLATFORM_PERIODIC_READY;
}

void RobotSafetyRequestDisable(void)
{
    TEST_CHECK(expected_monitor == TASK_MONITOR_MOTOR && timing_faults == 0u);
    if (before_send_budget || before_send_deadline) TEST_CHECK(motor_sends == 0u); // 耗尽时限时尚未推进发送队列。
    TEST_CHECK(waits == (finish_error ? 2u : 3u)); // 完成超期在当轮报告，漏拍在下一次等待时报告。
    ++stops; // 整车许可必须在核心故障上报前撤销。
}
#endif

#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
void RobotSafetyRequestDisable(void)
{
    TEST_CHECK(0); // 受限命令任务的故障入口还须撤销已发布的话题。
}

void RobotStopCommissioningCommand(void)
{
    TEST_CHECK(expected_monitor == TASK_MONITOR_ROBOT && work == 1u && timing_faults == 0u);
    ++stops; // 人工许可和旧命令必须在监控故障报告前撤销。
}
#endif

uint8_t TaskMonitorRuntimeReleased(void)
{
    return (uint8_t)(release_delays != 0u); // 初次让出处理器后才由协调者放行。
}

void PlatformTaskDelayMs(uint32_t milliseconds)
{
    if (release_delays == 0u)
    {
        TEST_CHECK(milliseconds == 1u && initializations == 0u && work == 0u);
        ++release_delays;
        return; // 放行之前，本任务只让出处理器。
    }
    if (!finish_error && waits == 2u)
    {
        TEST_CHECK(milliseconds == 1u && finishes == 1u && completes == 1u && fairness_delays == 0u);
        ++fairness_delays;
        return; // 正好在下一拍完成时，先让其他任务取得处理器。
    }
    // 发生错误后，仅可让出处理器。完成检查失败时不会报告已完成的工作。
    TEST_CHECK(milliseconds == expected_period_ms);
    TEST_CHECK(waits == (finish_error ? 2u : 3u) && completes == (finish_error ? 0u : 1u));
#if MC02_TASK_CONTROL || defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    TEST_CHECK(stops == hard_deadline && timing_faults == hard_deadline); // 硬期限任务先清理和报告故障，再进入停等。
#endif
    if (++fault_delays == 2u) longjmp(task_exit, 1); // 验证错误连续保持两轮后退出常驻循环。
}

PlatformPeriodicResult_t PlatformTaskPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config)
{
    TEST_CHECK(clock != NULL && initializations == 0u && release_delays == 1u);
    TEST_CHECK(config->period_us == expected_period_ms * 1000u && config->budget_us == config->period_us);
    clock->period_ticks = expected_period_ms;
    clock->budget_ticks = expected_period_ms; // 这里的 RTOS 替身使用与周期相同的预算。
    ++initializations;
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformTaskPeriodicFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, PlatformPeriodicTiming_t *timing)
{
    TEST_CHECK(work == 1u && waits == 2u && begins == 2u);
    TEST_CHECK(release->scheduled_tick == 0u); // 完成检查和等待共用原定拍点。
    ++finishes;
#if MC02_TASK_MOTOR_SPLIT
    if (expected_monitor == TASK_MONITOR_MOTOR && finishes == 1u)
    {
        TEST_CHECK(motor_sends == 0u); // 第一次复核必须在发送服务前进行。
        *timing = (PlatformPeriodicTiming_t){
            .work_ticks = before_send_budget ? clock->budget_ticks : before_send_deadline, // 分别正好到预算或绝对截止点。
        };
        return PLATFORM_PERIODIC_READY;
    }
    TEST_CHECK(finishes == (expected_monitor == TASK_MONITOR_MOTOR ? 2u : 1u));
#else
    TEST_CHECK(finishes == 1u);
#endif
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS) || MC02_TASK_CONTROL
    if (budget_error)
    {
        const uint32_t work = clock->budget_ticks + 1u; // 模拟多执行一个计数刻度。
        *timing = (PlatformPeriodicTiming_t){.work_ticks = work, .over_budget = 1u,
            .missed_deadline = (uint8_t)(release->observed_tick + work > clock->period_ticks)};
        return PLATFORM_PERIODIC_READY;
    }
    if (deadline_error)
    {
        *timing = (PlatformPeriodicTiming_t){.work_ticks = clock->period_ticks, .missed_deadline = 1u}; // 迟到使本拍错过截止点。
        return PLATFORM_PERIODIC_READY; // 时钟正常，任务自己报告已确认的超期。
    }
#endif
    if (finish_error) return PLATFORM_PERIODIC_CLOCK; // 事件已运行，但时钟失效使完成时刻不可信。
    *timing = (PlatformPeriodicTiming_t){.work_ticks = clock->period_ticks - release->observed_tick}; // 恰好到下一拍。
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformTaskPeriodicWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    TEST_CHECK(clock != NULL && release != NULL && initializations == 1u && waits < 3u);
    ++waits;
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS) || MC02_TASK_CONTROL
    if (waits == 3u && skipped_release)
    {
        *release = (PlatformPeriodicRelease_t){.skipped = 1u}; // 最近拍已就绪，但上一拍未执行。
        return PLATFORM_PERIODIC_READY; // 在任务领取监控许可之前发现漏拍。
    }
#endif
    if (waits == 3u) return PLATFORM_PERIODIC_CLOCK; // 模拟唤醒失败；之后不得再调用此接口。
    *release = (PlatformPeriodicRelease_t){0};
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS) || MC02_TASK_CONTROL
    if (waits == 2u && deadline_error) release->observed_tick = 1u; // 工作满足预算，但启动延迟使其超过完成期限。
#endif
    return PLATFORM_PERIODIC_READY;
}

#if MC02_TASK_CONTROL
static PlatformPeriodicClock_t *tim5_owner; // 硬件后端按实际任务栈地址记录所有权。
static unsigned tim5_closes;

PlatformPeriodicResult_t PlatformTaskHighResOpen(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config)
{
    TEST_CHECK(clock != NULL && tim5_owner == NULL && release_delays == 1u && initializations == 0u);
    TEST_CHECK(config->period_us == 500u && config->budget_us == 250u); // 不把实际周期向上取整。
    tim5_owner = clock; // 后续接口须传入同一个地址。
    clock->period_ticks = 500u;
    clock->budget_ticks = 250u; // TIM5 替身用微秒表达预算。
    ++initializations;
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformTaskHighResWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    TEST_CHECK(clock == tim5_owner && tim5_closes == 0u);
    const PlatformPeriodicResult_t result = PlatformTaskPeriodicWait(clock, release); // 使用相同的假等待事件。
    if (result == PLATFORM_PERIODIC_READY && waits == 2u && !budget_error)
        release->observed_tick = 250u; // 晚 250 μs 开始，工作 250 μs 恰好满足预算和完成期限。
    return result;
}

PlatformPeriodicResult_t PlatformTaskHighResFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, PlatformPeriodicTiming_t *timing)
{
    TEST_CHECK(clock == tim5_owner && tim5_closes == 0u);
    return PlatformTaskPeriodicFinish(clock, release, timing); // 复用模拟的到期和超期结果。
}

PlatformPeriodicResult_t PlatformTaskHighResClose(const PlatformPeriodicClock_t *clock)
{
    TEST_CHECK(clock == tim5_owner && stops == 1u && tim5_closes == 0u); // 撤销轮组输出后才能释放。
    ++tim5_closes;
    tim5_owner = NULL;
    return PLATFORM_PERIODIC_READY;
}
#endif

TaskMonitor_Result_e TaskMonitorReportTimingFailure(TaskMonitor_Id_e id, TaskMonitor_FaultReason_e reason)
{
#if MC02_TASK_CONTROL
    TEST_CHECK(id == expected_monitor && reason == expected_fault && timing_faults == 0u);
    TEST_CHECK(stops == 1u && tim5_owner == NULL); // 核心关断入口之前已撤销输出并释放比较通道。
    ++timing_faults;
#elif defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
    TEST_CHECK(id == TASK_MONITOR_ROBOT && reason == expected_fault && timing_faults == 0u);
    TEST_CHECK(stops == 1u && begins == 2u && work == 1u); // 已撤销许可，漏掉的拍没有触发工作。
    ++timing_faults;
#else
    (void)id;
    (void)reason;
    TEST_CHECK(0); // 其他常驻任务仍由监控轮询发现停顿。
#endif
    return TASK_MONITOR_RESULT_BUSY; // 模拟监控已锁存时仍不得重启拍点；真实首次关断在独立联测中验证。
}

TaskMonitor_Result_e TaskMonitorBeginWork(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == expected_monitor && waits == begins + 1u);
    return ++begins == 1u ? TASK_MONITOR_RESULT_BUSY : TASK_MONITOR_RESULT_OK; // 首拍即使到期也暂不许可。
}

TaskMonitor_Result_e TaskMonitorCompleteWork(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == expected_monitor && work == 1u && completes == 0u);
    ++completes;
    return TASK_MONITOR_RESULT_OK;
}

TaskMonitor_Result_e TaskMonitorSetWaiting(TaskMonitor_Id_e id, TaskMonitor_WaitReason_e reason)
{
    TEST_CHECK(id == expected_monitor && reason == TASK_MONITOR_WAIT_PERIOD && completes == 1u);
    ++idle_reports;
    return TASK_MONITOR_RESULT_OK;
}

static void RunWork(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == expected_monitor && waits == 2u && begins == 2u && work == 0u);
    ++work; // 只在第二次取得拍点且被监控许可时执行。
}

void INS_Task(void) { RunWork(TASK_MONITOR_INS); }
void MotorControlTask(void) { RunWork(TASK_MONITOR_MOTOR); }
void RobotTask(void) { RunWork(TASK_MONITOR_ROBOT); }
void DaemonTask(void) { RunWork(TASK_MONITOR_DAEMON); }
#if MC02_TASK_CHASSIS
void ChassisTask(void) { RunWork(TASK_MONITOR_CHASSIS); }
#endif
#if MC02_TASK_GIMBAL
void GimbalUpdateFeedback(void)
{
    TEST_CHECK(expected_monitor == TASK_MONITOR_GIMBAL && begins == 2u && work == 0u);
    ++gimbal_feedback; // 只有取得云台自己的拍点后才允许发布反馈。
}
void GimbalTask(void)
{
    TEST_CHECK(gimbal_feedback == 1u); // 先取得反馈，再执行该轮业务。
    RunWork(TASK_MONITOR_GIMBAL);
}
#endif
#if MC02_TASK_SHOOTER
void ShootTask(void) { RunWork(TASK_MONITOR_SHOOTER); }
#endif
void TaskMonitorTick(void) { TEST_CHECK(expected_monitor == TASK_MONITOR_DAEMON && work == 1u); }
void PlatformMaintainSpi(void) { TEST_CHECK(expected_monitor == TASK_MONITOR_DAEMON && begins == 2u); }
void PlatformMaintainAnalog(void) { TEST_CHECK(expected_monitor == TASK_MONITOR_DAEMON && begins == 2u); }
void PlatformMaintainCan(void) { TEST_CHECK(expected_monitor == TASK_MONITOR_DAEMON && work == 1u); }
uint32_t PlatformProbeStart(void) { return 42u; }
float PlatformProbeTimelineMs(void) { return 0.0f; }
void PlatformProbeDone(volatile PlatformProbe_t *probe, uint32_t start)
{
    TEST_CHECK(probe != NULL && start == 42u && work == 1u);
    probe->last_us = 0u;
}

int main(int argc, char **argv)
{
    static const struct {
        const char *name; // CTest 场景名。
        PlatformTaskEntry_t entry; // 固件实际使用的周期入口。
        TaskMonitor_Id_e monitor; // 入口应报告的监控项。
        uint32_t period_ms; // 入口应从配置取得的名义周期。
    } tasks[] = {
#if MC02_TASK_CONTROL
#if MC02_TASK_CHASSIS
        {"chassis", StartCHASSISTASK, TASK_MONITOR_CHASSIS, 4u}, // 正式任务入口使用隔离的主机时间配置。
        {"tim5", StartCHASSISTASK, TASK_MONITOR_CHASSIS, 1u}, // 错误后让出 1 ms，实际拍点为 500 μs。
#endif
#if MC02_TASK_GIMBAL
        {"gimbal", StartGIMBALTASK, TASK_MONITOR_GIMBAL, 4u}, // 云台在独立拍点发布反馈和计算。
        {"gimbal-tim5", StartGIMBALTASK, TASK_MONITOR_GIMBAL, 1u}, // TIM5 超预算时释放该任务的通道。
#endif
#if MC02_TASK_SHOOTER
        {"shooter", StartSHOOTERTASK, TASK_MONITOR_SHOOTER, 4u}, // 发射模块任务独立报告进度。
#endif
#if MC02_TASK_MOTOR_SPLIT
        {"motor", StartMOTORTASK, TASK_MONITOR_MOTOR, 1u}, // 计时错误时先撤销整车许可。
#endif
#else
        {"comm", StartMOTORTASK, TASK_MONITOR_MOTOR, 1u}, // 禁热版本只派发通信。
#endif
        {"ins", StartINSTASK, TASK_MONITOR_INS, 1u}, // 姿态采样与解算。
        {"daemon", StartDAEMONTASK, TASK_MONITOR_DAEMON, 10u}, // 维护及监控。
        {"robot", StartROBOTTASK, TASK_MONITOR_ROBOT, 5u}, // 整车诊断。
    };
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS) || MC02_TASK_CONTROL
    skipped_release = (uint8_t)(argc == 3 && strcmp(argv[2], "skip-release") == 0);
    deadline_error = (uint8_t)(argc == 3 && strcmp(argv[2], "deadline-error") == 0);
    budget_error = (uint8_t)(argc == 3 && strcmp(argv[2], "budget-error") == 0);
#if MC02_TASK_MOTOR_SPLIT
    before_send_budget = (uint8_t)(argc == 3 && strcmp(argv[2], "pre-send-budget") == 0);
    before_send_deadline = (uint8_t)(argc == 3 && strcmp(argv[2], "pre-send-deadline") == 0);
    budget_error |= before_send_budget; // 验证安全报告将耗尽预算与时钟失效分开。
    deadline_error |= before_send_deadline; // 让等待替身模拟一个刻度的迟醒。
#endif
    TEST_CHECK(argc == 2 || (argc == 3 && (skipped_release || deadline_error ||
        strcmp(argv[2], "finish-error") == 0 || budget_error)));
    finish_error = (uint8_t)(argc == 3 && !skipped_release); // 跳拍在第一轮成功完成后注入。
    if (skipped_release || deadline_error) expected_fault = TASK_MONITOR_FAULT_PERIOD_DEADLINE;
    if (budget_error) expected_fault = TASK_MONITOR_FAULT_WORK_DEADLINE; // 执行预算超期优先报告。
#else
    TEST_CHECK(argc == 2 || (argc == 3 && (strcmp(argv[2], "finish-error") == 0 || strcmp(argv[2], "budget-error") == 0)));
    finish_error = (uint8_t)(argc == 3);
#endif
    for (size_t i = 0u; i < sizeof(tasks) / sizeof(tasks[0]); ++i)
    {
        if (strcmp(tasks[i].name, argv[1]) != 0) continue;
        expected_monitor = tasks[i].monitor;
        expected_period_ms = tasks[i].period_ms;
#if MC02_TASK_MOTOR_SPLIT
        if (before_send_deadline) expected_period_ms = 2u; // 留出独立于执行预算的绝对截止点场景。
#endif
#if defined(MC02_PROFILE_COMMISSIONING_CHASSIS)
        hard_deadline = expected_monitor == TASK_MONITOR_ROBOT; // 受限配置中的整车任务发布人工命令。
#elif MC02_TASK_CONTROL
        hard_deadline = expected_monitor != TASK_MONITOR_INS && expected_monitor != TASK_MONITOR_DAEMON &&
            expected_monitor != TASK_MONITOR_ROBOT; // 正式配置区分独立控制与普通常驻任务。
#endif
#if MC02_TASK_CONTROL
        const uint8_t tim5_selected = (uint8_t)(strcmp(argv[1], "tim5") == 0 || strcmp(argv[1], "gimbal-tim5") == 0);
        if (tim5_selected)
        {
            control.timing = (PlatformPeriodicConfig_t){.period_us = 500u, .budget_us = 250u};
            control.source = PLATFORM_PERIODIC_SOURCE_HIGH_RES; // 模拟启动预检交付的实际来源。
        }
#endif
        if (setjmp(task_exit) == 0) tasks[i].entry(NULL);
        unsigned expected_finishes = 1u; // 普通任务仅在本轮结束时复核。
#if MC02_TASK_MOTOR_SPLIT
        if (expected_monitor == TASK_MONITOR_MOTOR && !before_send_budget && !before_send_deadline)
            expected_finishes = 2u; // 正常进入发送时仍需在轮末再次检查。
#endif
        TEST_CHECK(initializations == 1u && begins == 2u && work == 1u && finishes == expected_finishes && fault_delays == 2u);
        TEST_CHECK(waits == (finish_error ? 2u : 3u));
        const unsigned reported = finish_error ? 0u : 1u; // 完成时钟失败时保持监控进度不变。
        TEST_CHECK(completes == reported && idle_reports == reported);
        TEST_CHECK(fairness_delays == (hard_deadline ? 0u : reported)); // 硬期限任务按原拍点等待，普通任务主动让出一个刻度。
#if MC02_TASK_CONTROL
        TEST_CHECK(tim5_closes == tim5_selected && tim5_owner == NULL); // 故障后只释放一次硬件通道。
#endif
        return 0;
    }
    TEST_CHECK(0);
    return 1;
}
