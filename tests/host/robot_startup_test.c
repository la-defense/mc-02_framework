/** @file robot_startup_test.c
 * @brief 注入启动步骤的结果，核对生产协调器的调用顺序和失能行为。
 */
#include "robot_startup.h"
#include "ins_startup.h"
#include "config/vehicle_selection.h"
#include "task_monitor.h"
#include "init_diagnostic.h"
#include "test_check.h"
#include <stdint.h>
#include <string.h>

static char calls[32]; // 按实际顺序记录设备、任务、看门狗与放行步骤。
static unsigned count, faults, disabled, shutdowns, failures, ready, stage_bits;
static uint8_t scheduler = 1u, enabled = 1u, application_ok = 1u, begin_ok = 1u, tasks_ok = 1u;
static uint8_t end_ok = 1u, watchdog_ok = 1u, release_ok = 1u;
static RobotInsStartupResult_e ins_result = ROBOT_INS_STARTUP_READY;
static InitDiagnosticStage_e failure_stage;
static InitDiagnosticReason_e failure_reason;
static uint8_t failure_module;
static uint16_t failure_resource;

static void Record(char value) { TEST_CHECK(count + 1u < sizeof(calls)); calls[count++] = value; }
uint8_t PlatformSchedulerIsRunning(void) { return scheduler; }
uint8_t RobotInitializeApplications(void) { Record('A'); return application_ok; }
void RobotRunStartupDiagnostics(void) { Record('K'); } // 诊断必须位于设备之后、任务创建之前。
uint8_t RobotBuildModuleStarted(RobotModule_t module) { TEST_CHECK(module == ROBOT_MODULE_IMU); return enabled; }
RobotInsStartupResult_e RobotInitializeIns(void) { Record('I'); return ins_result; }
uint8_t PlatformTaskBatchBegin(void) { Record('B'); return begin_ok; }
uint8_t PlatformTaskBatchEnd(void) { Record('E'); return end_ok; }
uint8_t OSTaskInit(void) { Record('T'); return tasks_ok; }
uint8_t BSP_WatchdogInitChecked(uint32_t timeout) { TEST_CHECK(timeout == 200u); Record('W'); return watchdog_ok; }
TaskMonitor_Result_e TaskMonitorReleaseRuntime(void)
{ Record('R'); return release_ok ? TASK_MONITOR_RESULT_OK : TASK_MONITOR_RESULT_BUSY; }
void InitDiagnosticSetStage(InitDiagnosticStage_e stage)
{
    TEST_CHECK(stage == INIT_STAGE_APPLICATION || stage == INIT_STAGE_TASKS); // 仅接受实际启动阶段。
    stage_bits |= 1u << (unsigned)stage; // 保存到达的阶段，供各失败路径核对。
}
uint8_t InitDiagnosticIsFailed(void) { return (uint8_t)(failures != 0u); }
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    failure_stage = stage; // 保留协调器报告的失败阶段。
    failure_module = module; // 保留出错模块供断言。
    failure_reason = reason; // 保留错误类别供断言。
    failure_resource = resource; // 保留出错资源供断言。
    ++failures; // 统计报告次数。
    Record('F'); // 检查诊断与关断的相对顺序。
}
void InitDiagnosticMarkReady(void) { ++ready; Record('Y'); }
void RobotFaultReport(unsigned fault) { TEST_CHECK(fault == 1u); ++faults; Record('X'); }
void RobotSafetyRequestDisable(void) { ++disabled; Record('D'); }
void BSP_SafetyStartupLatchOutputsOff(void) { ++shutdowns; Record('O'); }

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个场景单独运行，保留生产代码的一次性状态。
    const char *expected = "AIKBTEWRY"; // 正常路径先运行诊断，再创建并放行任务。
    uint8_t success = 1u; // 只有成功与允许降级的场景应完成启动。
    if (!strcmp(argv[1], "disabled")) { enabled = 0u; expected = "AKBTEWRY"; }
    else if (!strcmp(argv[1], "aux")) ins_result = ROBOT_INS_STARTUP_AUXILIARY_FAILURE;
    else if (!strcmp(argv[1], "core")) { ins_result = ROBOT_INS_STARTUP_CORE_FAILURE; expected = "AIFXDO"; success = 0u; }
    else if (!strcmp(argv[1], "application")) { application_ok = 0u; expected = "AFXDO"; success = 0u; }
    else if (!strcmp(argv[1], "no-scheduler")) { scheduler = 0u; expected = "FXDO"; success = 0u; }
    else if (!strcmp(argv[1], "batch-begin")) { begin_ok = 0u; expected = "AIKBFXDO"; success = 0u; }
    else if (!strcmp(argv[1], "tasks")) { tasks_ok = 0u; expected = "AIKBTFXDOE"; success = 0u; }
    else if (!strcmp(argv[1], "batch-end")) { end_ok = 0u; expected = "AIKBTEFXDO"; success = 0u; }
    else if (!strcmp(argv[1], "watchdog")) { watchdog_ok = 0u; expected = "AIKBTEWFXDO"; success = 0u; }
    else if (!strcmp(argv[1], "release")) { release_ok = 0u; expected = "AIKBTEWRFXDO"; success = 0u; }
    else TEST_CHECK(!strcmp(argv[1], "success"));
    TEST_CHECK(RobotStartupRun() == success); // 返回值应反映本次核心步骤是否成功。
    TEST_CHECK(!strcmp(calls, expected)); // 创建失败必须在恢复调度前关断。
    TEST_CHECK(ready == success && faults == !success && disabled == !success && shutdowns == !success);
    TEST_CHECK(((stage_bits >> INIT_STAGE_APPLICATION) & 1u) == scheduler); // 传感器等待属于业务初始化阶段。
    TEST_CHECK(((stage_bits >> INIT_STAGE_TASKS) & 1u) ==
               (scheduler && application_ok && ins_result != ROBOT_INS_STARTUP_CORE_FAILURE)); // 核心失败不能进入创建任务阶段。
    if (!watchdog_ok)
    {
        TEST_CHECK(failure_stage == INIT_STAGE_TASKS && failure_module == INIT_MODULE_WATCHDOG);
        TEST_CHECK(failure_reason == INIT_FAILURE_WATCHDOG_CONFIG &&
                   failure_resource == INIT_RESOURCE_WATCHDOG_RUNTIME); // 看门狗失败明确报告具体窗口。
    }
    if (!tasks_ok)
        TEST_CHECK(failure_stage == INIT_STAGE_TASKS && failure_module == INIT_MODULE_TASKS &&
                   failure_reason == INIT_FAILURE_TASK_CREATE); // 创建失败报告任务阶段。
    TEST_CHECK(!RobotStartupRun()); // 重复调用不能重新初始化、放行或清除失败。
    TEST_CHECK(!strcmp(calls, expected)); // 第二次调用不产生任何硬件操作。
    return 0;
}
