/** @file robot_task_create_test.c
 * @brief 验证真实任务创建流程保留任务配置及核心／辅助失败处理。
 */
#include "test_check.h"
#include "robot_task_create.h"
#include "config/vehicle_selection.h"
#include "init_diagnostic.h"
#include "platform_os.h"
#include "platform_uart.h"
#include "bsp_log.h"
#include "daemon.h"
#include <stdint.h>
#include <string.h>

static const char *failure_name;
static uint8_t ui_started = 1u, lcd_started = 1u, uart_ready = 1u;
static RobotModule_t core_failure = ROBOT_MODULE_COUNT;
static unsigned created, failures, degraded;
static uint16_t last_failure, last_degraded;
static int handle_storage[6];
static DaemonStatus_t daemon_result = DAEMON_OK;
static BSPLogStatus_t log_result = BSP_LOG_STATUS_OK;

void StartINSTASK(void const *arg) {(void)arg;}
void StartMOTORTASK(void const *arg) {(void)arg;}
void StartDAEMONTASK(void const *arg) {(void)arg;}
void StartROBOTTASK(void const *arg) {(void)arg;}
void StartUITASK(void const *arg) {(void)arg;}
void StartLCDTASK(void const *arg) {(void)arg;}

uint8_t PlatformTaskCreateTracked(const PlatformTaskConfig_t *config, PlatformTaskHandle_t *handle)
{
    static const struct {const char *name; uint32_t stack; PlatformTaskPriority_t priority;} expected[] = {
        {"instask", 1024u, PLATFORM_TASK_ABOVE_NORMAL}, // 姿态任务需要较高优先级。
        {"motortask", 512u, PLATFORM_TASK_NORMAL}, // 电机及通信派发任务。
        {"daemontask", 512u, PLATFORM_TASK_NORMAL}, // 状态检查任务。
        {"robottask", 1024u, PLATFORM_TASK_NORMAL}, // 整车控制任务。
        {"uitask", 512u, PLATFORM_TASK_NORMAL}, // 裁判图形任务。
        {"lcdtask", 1024u, PLATFORM_TASK_LOW}, // LCD 刷新任务。
    };
    unsigned index = 0u;
    while (index < 6u && strcmp(config->name, expected[index].name) != 0) ++index; // 根据名称核对对应配置。
    TEST_CHECK(index < 6u && config->entry != NULL && config->argument == NULL);
    TEST_CHECK(config->stack_words == expected[index].stack && config->priority == expected[index].priority);
    ++created; // 即使后端拒绝，也记录框架尝试创建的任务数。
    if (failure_name != NULL && strcmp(config->name, failure_name) == 0) return 0u;
    *handle = &handle_storage[index]; // 返回稳定的模拟句柄，不执行任务入口。
    return 1u;
}

uint8_t PlatformUartStartTxTask(void) {return uart_ready;}
BSPLogStatus_t BSPLogTaskInit(void) {return log_result;}
DaemonStatus_t DaemonEventTaskInit(void) {return daemon_result;}
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    return module == ROBOT_MODULE_UI ? ui_started : module == ROBOT_MODULE_LCD ? lcd_started : 0u;
}
uint8_t RobotBuildMarkFailure(RobotModule_t module) {return (uint8_t)(module == core_failure);}
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK(stage == INIT_STAGE_TASKS && module == INIT_MODULE_TASKS && reason == INIT_FAILURE_TASK_CREATE);
    ++failures;
    last_failure = resource;
}
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK(stage == INIT_STAGE_TASKS && module == INIT_MODULE_TASKS && reason == INIT_FAILURE_TASK_CREATE);
    ++degraded;
    last_degraded = resource;
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "core-ins") == 0) failure_name = "instask";
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
    if (failure_name != NULL && strcmp(failure_name, "instask") == 0)
        TEST_CHECK(result == 0u && failures == 1u && last_failure == INIT_RESOURCE_TASK_INS);
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
    TEST_CHECK(created == 4u + ui_started + lcd_started); // 未启用的辅助模块不创建任务。
    return 0;
}
