/** @file robot_task_create_test.c
 * @brief 验证真实任务创建流程保留任务配置及核心／辅助失败处理。
 */
#include "test_check.h"
#include "robot_task_create.h"
#include "config/build_selection.h"
#include "config/vehicle_selection.h"
#include "init_diagnostic.h"
#include "platform_os.h"
#include "platform_uart.h"
#include "bsp_log.h"
#include "daemon.h"
#include "task_monitor.h"
#if MC02_TASK_CHASSIS
#include "robot_monitor_plan.h"
#endif
#include <stdint.h>
#include <string.h>

static const char *failure_name;
static uint8_t ui_started = 1u, lcd_started = 1u, imu_started = 1u, uart_ready = 1u;
static uint8_t remove_monitor_ok = 1u; // 模拟辅助 INS 监控能否安全撤销。
static RobotModule_t core_failure = ROBOT_MODULE_COUNT;
static unsigned created, failures, degraded;
static uint16_t last_failure, last_degraded;
static int handle_storage[7]; // 包括最多一个独立底盘任务。
static DaemonStatus_t daemon_result = DAEMON_OK;
static BSPLogStatus_t log_result = BSP_LOG_STATUS_OK;
#if defined(TEST_HT_TASK)
static uint8_t ht_result = 1u; // 专项场景控制 HT 创建结果。
static unsigned ht_calls;
uint8_t HTMotorControlInit(void) {++ht_calls; return ht_result;}
#endif

void StartINSTASK(void const *arg) {(void)arg;}
void StartMOTORTASK(void const *arg) {(void)arg;}
void StartDAEMONTASK(void const *arg) {(void)arg;}
void StartROBOTTASK(void const *arg) {(void)arg;}
void StartUITASK(void const *arg) {(void)arg;}
void StartLCDTASK(void const *arg) {(void)arg;}
#if MC02_TASK_CHASSIS
static uint8_t chassis_started = 1u, chassis_prepared = 1u;
static const RobotChassisTaskConfig_t chassis_config = {
    .stack_words = 192u, // 主机替身只核对传给创建接口的配置。
};
const RobotChassisTaskConfig_t *RobotGetChassisTaskConfig(void)
{
    return chassis_prepared ? &chassis_config : NULL;
}
void StartCHASSISTASK(void const *arg) {(void)arg;}
#endif

uint8_t PlatformTaskCreateTracked(const PlatformTaskConfig_t *config, PlatformTaskHandle_t *handle)
{
    static const struct {const char *name; uint32_t stack; PlatformTaskPriority_t priority;} expected[] = {
        {"instask", 1024u, PLATFORM_TASK_ABOVE_NORMAL}, // 姿态任务需要较高优先级。
        {"motortask", 512u, PLATFORM_TASK_NORMAL}, // 电机及通信派发任务。
        {"daemontask", 512u, PLATFORM_TASK_NORMAL}, // 状态检查任务。
        {"robottask", 1024u, PLATFORM_TASK_NORMAL}, // 整车控制任务。
        {"uitask", 512u, PLATFORM_TASK_NORMAL}, // 裁判图形任务。
        {"lcdtask", 1024u, PLATFORM_TASK_LOW}, // LCD 刷新任务。
#if MC02_TASK_CHASSIS
        {"chassistask", 192u, PLATFORM_TASK_NORMAL}, // 底盘独立任务使用试验栈字数。
#endif
    };
    const unsigned limit = sizeof(expected) / sizeof(expected[0]);
    unsigned index = 0u;
    while (index < limit && strcmp(config->name, expected[index].name) != 0) ++index; // 根据名称核对对应配置。
    TEST_CHECK(index < limit && config->entry != NULL && config->argument == NULL);
#if MC02_TASK_CHASSIS
    if (index == 6u) TEST_CHECK(config->entry == StartCHASSISTASK);
#endif
    TEST_CHECK(config->stack_words == expected[index].stack && config->priority == expected[index].priority);
    ++created; // 即使后端拒绝，也记录框架尝试创建的任务数。
    if (failure_name != NULL && strcmp(config->name, failure_name) == 0) return 0u;
    *handle = &handle_storage[index]; // 返回稳定的模拟句柄，不执行任务入口。
    return 1u;
}

uint8_t PlatformUartStartTxTask(void) {return uart_ready;}
TaskMonitor_Result_e TaskMonitorRemoveDeferredAuxiliary(TaskMonitor_Id_e id)
{
    TEST_CHECK(id == TASK_MONITOR_INS); // 只允许撤销失败的 INS 监控。
    return remove_monitor_ok ? TASK_MONITOR_RESULT_OK : TASK_MONITOR_RESULT_BUSY;
}
BSPLogStatus_t BSPLogTaskInit(void) {return log_result;}
DaemonStatus_t DaemonEventTaskInit(void) {return daemon_result;}
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
#if MC02_TASK_CHASSIS
    if (module == ROBOT_MODULE_CHASSIS) return chassis_started;
#endif
    return module == ROBOT_MODULE_UI ? ui_started : module == ROBOT_MODULE_LCD ? lcd_started :
           module == ROBOT_MODULE_IMU ? imu_started : 0u; // 分别模拟模块启动结果。
}
uint8_t RobotBuildMarkFailure(RobotModule_t module) {return (uint8_t)(module == core_failure);}
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK((stage == INIT_STAGE_TASKS && module == INIT_MODULE_TASKS &&
                (reason == INIT_FAILURE_TASK_CREATE || reason == INIT_FAILURE_DEPENDENCY)) ||
               (stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_INS && reason == INIT_FAILURE_RESOURCE));
    ++failures;
    last_failure = resource;
}
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK((stage == INIT_STAGE_TASKS && module == INIT_MODULE_TASKS && reason == INIT_FAILURE_TASK_CREATE) ||
               (stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_INS && reason == INIT_FAILURE_RESOURCE));
    ++degraded;
    last_degraded = resource;
}

#if MC02_TASK_CHASSIS
static int TestChassisCreation(const char *scenario)
{
    uint8_t success = 1u;
    unsigned expected_created = 7u;
    if (strcmp(scenario, "chassis-create") == 0) { failure_name = "chassistask"; success = 0u; }
    else if (strcmp(scenario, "chassis-missing") == 0)
    { chassis_prepared = 0u; success = 0u; expected_created = 6u; }
    else if (strcmp(scenario, "chassis-not-started") == 0)
    { chassis_started = 0u; success = 0u; expected_created = 6u; }
    else if (strcmp(scenario, "chassis-disabled") == 0)
    { chassis_prepared = 0u; chassis_started = 0u; expected_created = 6u; }
    else TEST_CHECK(strcmp(scenario, "chassis-normal") == 0);

    TEST_CHECK(OSTaskInit() == success && created == expected_created && failures == (unsigned)!success);
    if (!success) TEST_CHECK(last_failure == INIT_RESOURCE_TASK_CHASSIS); // 未就绪时不可放行整批任务。
    return 0;
}
#endif

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
#if MC02_TASK_CHASSIS
    if (strncmp(argv[1], "chassis-", 8u) == 0) return TestChassisCreation(argv[1]);
#endif
    if (strcmp(argv[1], "heater-core") == 0 || strcmp(argv[1], "heater-aux") == 0)
    {
        core_failure = strcmp(argv[1], "heater-core") == 0 ? ROBOT_MODULE_HEATER : ROBOT_MODULE_COUNT;
        uint8_t core = RobotReportHeaterInitFailure(); // 不创建任务，单独检查初始化故障分类。
        TEST_CHECK(core == (core_failure == ROBOT_MODULE_HEATER));
        TEST_CHECK(failures == core && degraded == (unsigned)!core);
        return 0;
    }
    if (strcmp(argv[1], "ht-failure") == 0) {
#if defined(TEST_HT_TASK)
        ht_result = 0u; // 测试 HT 创建失败传递给整车。
#else
        TEST_CHECK(0 && "HT 测试未启用");
#endif
    }
    else if (strcmp(argv[1], "core-ins") == 0) { failure_name = "instask"; core_failure = ROBOT_MODULE_IMU; }
    else if (strcmp(argv[1], "aux-ins") == 0) failure_name = "instask";
    else if (strcmp(argv[1], "aux-ins-monitor-fail") == 0) { failure_name = "instask"; remove_monitor_ok = 0u; }
    else if (strcmp(argv[1], "disabled-ins") == 0) imu_started = 0u;
    else if (strcmp(argv[1], "disabled-ins-monitor-fail") == 0) { imu_started = 0u; remove_monitor_ok = 0u; }
    else if (strcmp(argv[1], "core-motor") == 0) failure_name = "motortask";
    else if (strcmp(argv[1], "core-daemon") == 0) failure_name = "daemontask";
    else if (strcmp(argv[1], "core-robot") == 0) failure_name = "robottask";
    else if (strcmp(argv[1], "aux-ui") == 0) failure_name = "uitask";
    else if (strcmp(argv[1], "core-ui") == 0) {failure_name = "uitask"; core_failure = ROBOT_MODULE_UI;}
    else if (strcmp(argv[1], "disabled-ui") == 0) ui_started = 0u;
    else if (strcmp(argv[1], "disabled-lcd") == 0) lcd_started = 0u;
    else if (strcmp(argv[1], "aux-lcd") == 0) failure_name = "lcdtask";
    else if (strcmp(argv[1], "core-lcd") == 0) {failure_name = "lcdtask"; core_failure = ROBOT_MODULE_LCD;}
    else if (strcmp(argv[1], "uart") == 0) uart_ready = 0u;
    else if (strcmp(argv[1], "event") == 0) daemon_result = DAEMON_ERR_TASK_CREATE;
    else if (strcmp(argv[1], "no-event") == 0) daemon_result = DAEMON_NO_TASK_REQUIRED;
    else if (strcmp(argv[1], "log") == 0) log_result = BSP_LOG_STATUS_TASK_CREATE_FAILED;
    else if (strcmp(argv[1], "core-log") == 0) {log_result = BSP_LOG_STATUS_TASK_CREATE_FAILED; core_failure = ROBOT_MODULE_LOG;}
    else TEST_CHECK(strcmp(argv[1], "normal") == 0);

    uint8_t result = OSTaskInit(); // 入口只创建模拟任务，不启动执行器。
    if (strcmp(argv[1], "ht-failure") == 0)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_MOTOR);
    else if (!imu_started && !remove_monitor_ok)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_INS);
    else if (core_failure == ROBOT_MODULE_IMU)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_INS);
    else if (failure_name != NULL && strcmp(failure_name, "instask") == 0 && !remove_monitor_ok)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_INS);
    else if (failure_name != NULL && strcmp(failure_name, "instask") == 0)
        TEST_CHECK(result == 1u && degraded == 1u && last_degraded == INIT_RESOURCE_TASK_INS);
    else if (failure_name != NULL && strcmp(failure_name, "motortask") == 0)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_MOTOR);
    else if (failure_name != NULL && strcmp(failure_name, "daemontask") == 0)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_DAEMON);
    else if (failure_name != NULL && strcmp(failure_name, "robottask") == 0)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_ROBOT);
    else if (core_failure == ROBOT_MODULE_UI)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_UI);
    else if (core_failure == ROBOT_MODULE_LCD)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_LCD);
    else if (core_failure == ROBOT_MODULE_LOG)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_LOG);
    else if (!uart_ready)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_UART_TX);
    else if (daemon_result != DAEMON_OK && daemon_result != DAEMON_NO_TASK_REQUIRED)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_DAEMON);
    else if (failure_name != NULL && strcmp(failure_name, "uitask") == 0)
        TEST_CHECK(result == 1u && degraded == 1u && last_degraded == INIT_RESOURCE_TASK_UI);
    else if (failure_name != NULL && strcmp(failure_name, "lcdtask") == 0)
        TEST_CHECK(result == 1u && degraded == 1u && last_degraded == INIT_RESOURCE_TASK_LCD);
    else if (log_result != BSP_LOG_STATUS_OK)
        TEST_CHECK(result == 1u && degraded == 1u && last_degraded == INIT_RESOURCE_TASK_LOG);
    else
        TEST_CHECK(result == 1u && failures == 0u && degraded == 0u);
    TEST_CHECK(created == 3u + imu_started + ui_started + lcd_started); // 未启用的模块不创建任务。
#if defined(TEST_HT_TASK)
    TEST_CHECK(ht_calls == (unsigned)(failure_name == NULL)); // 已有核心任务失败时不启动 HT。
#endif
    return 0;
}
