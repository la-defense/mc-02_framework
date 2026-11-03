/** @file alarm_startup_test.c
 * @brief 用生产启动计划检查告警降级的核心与辅助处理。
 */
#include "test_check.h"
#include "alarm_startup.h"
#include "init_diagnostic.h"
#include <stddef.h>

static RobotAlarmStatus_t alarm_status;
static unsigned degraded_events;
static RobotAlarmConfig_t alarm_config;
static RobotModulePlan_t plan;

RobotAlarmStatus_t RobotAlarmInit(const RobotAlarmConfig_t *config)
{
    TEST_CHECK(config == &alarm_config);
    return alarm_status;
}

void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource_id)
{
    TEST_CHECK(stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_ROBOT);
    TEST_CHECK(reason == INIT_FAILURE_RESOURCE && resource_id == ROBOT_MODULE_ALARM);
    ++degraded_events;
}

static uint8_t StartModule(void *context, unsigned module)
{
    (void)context;
    return module == ROBOT_MODULE_ALARM ? RobotStartAlarmConfigured(&alarm_config, &plan) : 1u;
}

int main(void)
{
    const uint32_t alarm = ROBOT_MODULE_BIT(ROBOT_MODULE_ALARM);
    const uint32_t chassis = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS);
    RobotModuleStartup_t state;
    plan.enabled = alarm | chassis; // 两个模块都参与启动。
    plan.required = chassis; // 告警初始为辅助模块。
    alarm_status = ROBOT_ALARM_DEGRADED;
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_OK);
    TEST_CHECK((state.started & alarm) != 0u && degraded_events == 1u);
    plan.dependencies[ROBOT_MODULE_CHASSIS] = alarm; // 核心底盘依赖告警时，告警进入核心闭包。
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK((state.failed & alarm) != 0u && degraded_events == 1u);
    alarm_status = ROBOT_ALARM_OK;
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_OK);
    TEST_CHECK((state.started & alarm) != 0u);
    alarm_status = ROBOT_ALARM_ERROR_CONFIG;
    TEST_CHECK(RobotStartAlarmConfigured(&alarm_config, &plan) == 0u);
    TEST_CHECK(RobotStartAlarmConfigured(NULL, &plan) == 0u);
    TEST_CHECK(RobotStartAlarmConfigured(&alarm_config, NULL) == 0u);
    return 0;
}
