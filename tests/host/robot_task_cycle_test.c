/** @file robot_task_cycle_test.c
 * @brief 从实际任务入口验证放行、监控许可和唤醒故障后的行为。
 */
#include "robot_task.h"
#include "platform_os.h"
#include "platform_periodic.h"
#include "platform_probe.h"
#include "task_monitor.h"
#include "test_check.h"
#include <setjmp.h>
#include <string.h>

static jmp_buf task_exit;
static TaskMonitor_Id_e expected_monitor;
static uint32_t expected_period_ms;
static unsigned release_delays, initializations, waits, begins, work, finishes, completes, idle_reports, fault_delays;
static unsigned fairness_delays;
static uint8_t finish_error; // 注入实际工作以后才发生的计数错误。

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
    if (++fault_delays == 2u) longjmp(task_exit, 1); // 验证错误连续保持两轮后退出常驻循环。
}

PlatformPeriodicResult_t PlatformTaskPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config)
{
    TEST_CHECK(clock != NULL && initializations == 0u && release_delays == 1u);
    TEST_CHECK(config->period_us == expected_period_ms * 1000u && config->budget_us == config->period_us);
    clock->period_ticks = expected_period_ms;
    ++initializations;
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformTaskPeriodicFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, PlatformPeriodicTiming_t *timing)
{
    TEST_CHECK(work == 1u && waits == 2u && begins == 2u && finishes == 0u);
    TEST_CHECK(release->scheduled_tick == 0u && release->observed_tick == 0u);
    ++finishes;
    if (finish_error) return PLATFORM_PERIODIC_CLOCK; // 事件已运行，但时钟失效使完成时刻不可信。
    *timing = (PlatformPeriodicTiming_t){.work_ticks = clock->period_ticks}; // 恰好到下一拍。
    return PLATFORM_PERIODIC_READY;
}

PlatformPeriodicResult_t PlatformTaskPeriodicWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release)
{
    TEST_CHECK(clock != NULL && release != NULL && initializations == 1u && waits < 3u);
    if (++waits == 3u) return PLATFORM_PERIODIC_CLOCK; // 模拟唤醒失败；之后不得再调用此接口。
    *release = (PlatformPeriodicRelease_t){0};
    return PLATFORM_PERIODIC_READY;
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
        {"ins", StartINSTASK, TASK_MONITOR_INS, 1u}, // 姿态采样与解算。
        {"comm", StartMOTORTASK, TASK_MONITOR_MOTOR, 1u}, // 禁热版本只派发通信。
        {"daemon", StartDAEMONTASK, TASK_MONITOR_DAEMON, 10u}, // 维护及监控。
        {"robot", StartROBOTTASK, TASK_MONITOR_ROBOT, 5u}, // 整车诊断。
    };
    TEST_CHECK(argc == 2 || (argc == 3 && strcmp(argv[2], "finish-error") == 0));
    finish_error = (uint8_t)(argc == 3);
    for (size_t i = 0u; i < sizeof(tasks) / sizeof(tasks[0]); ++i)
    {
        if (strcmp(tasks[i].name, argv[1]) != 0) continue;
        expected_monitor = tasks[i].monitor;
        expected_period_ms = tasks[i].period_ms;
        if (setjmp(task_exit) == 0) tasks[i].entry(NULL);
        TEST_CHECK(initializations == 1u && begins == 2u && work == 1u && finishes == 1u && fault_delays == 2u);
        TEST_CHECK(waits == (finish_error ? 2u : 3u));
        const unsigned reported = finish_error ? 0u : 1u; // 完成时钟失败时保持监控进度不变。
        TEST_CHECK(completes == reported && idle_reports == reported && fairness_delays == reported);
        return 0;
    }
    TEST_CHECK(0);
    return 1;
}
