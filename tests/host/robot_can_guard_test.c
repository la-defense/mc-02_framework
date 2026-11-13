/** @file robot_can_guard_test.c
 * @brief 使用生产依赖传播逻辑验证诊断后的 CAN 隔离分类。
 */
#include "robot_can_guard.h"
#include "config/vehicle_selection.h"
#include "platform_can.h"
#include "init_diagnostic.h"
#include "test_check.h"
#include <stddef.h>
#include <string.h>

static RobotModulePlan_t plan;
static RobotModuleStartup_t startup;
static uint32_t quarantined;
static unsigned failures, degraded;
static uint16_t recorded_bus;

uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    return (uint8_t)((startup.started & ROBOT_MODULE_BIT(module)) != 0u);
}
uint8_t RobotBuildMarkFailure(RobotModule_t module)
{
    return (uint8_t)(RobotModuleStartupMarkFailure(&plan, &startup, module) == ROBOT_MODULE_CORE_FAILURE);
}
uint8_t PlatformCanBusIsQuarantined(PlatformCanBus_t bus)
{
    return (uint8_t)((quarantined & (1u << (unsigned)bus)) != 0u);
}
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK(stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_ROBOT && reason == INIT_FAILURE_RESOURCE);
    recorded_bus = resource; ++failures;
}
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource)
{
    TEST_CHECK(stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_ROBOT && reason == INIT_FAILURE_RESOURCE);
    recorded_bus = resource; ++degraded;
}

int main(void)
{
    const uint32_t chassis = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS);
    const uint32_t gimbal = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL);
    const RobotCanDependency_t items[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1}, // 核心底盘使用第一条总线。
        {ROBOT_MODULE_GIMBAL, PLATFORM_CAN_BUS_2}, // 辅助云台使用第二条总线。
    };
    plan.enabled = chassis | gimbal; plan.required = chassis;
    startup.started = plan.enabled;
    quarantined = 1u << PLATFORM_CAN_BUS_2;
    TEST_CHECK(RobotCanGuardCheck(items, 2u)); // 辅助总线隔离允许核心继续。
    TEST_CHECK(degraded == 1u && failures == 0u && recorded_bus == PLATFORM_CAN_BUS_2);
    TEST_CHECK((startup.started & gimbal) == 0u && (startup.started & chassis) != 0u);
    memset(&startup, 0, sizeof(startup)); startup.started = plan.enabled;
    quarantined = 1u << PLATFORM_CAN_BUS_1;
    TEST_CHECK(!RobotCanGuardCheck(items, 2u)); // 核心总线隔离阻止任务放行。
    TEST_CHECK(failures == 1u && recorded_bus == PLATFORM_CAN_BUS_1 && startup.core_failed);
    memset(&startup, 0, sizeof(startup)); startup.started = plan.enabled;
    plan.dependencies[ROBOT_MODULE_CHASSIS] = gimbal; // 核心底盘间接依赖辅助云台。
    quarantined = 1u << PLATFORM_CAN_BUS_2;
    TEST_CHECK(!RobotCanGuardCheck(items, 2u)); // 依赖传播后仍属于核心故障。
    TEST_CHECK(failures == 2u && recorded_bus == PLATFORM_CAN_BUS_2 && startup.core_failed);
    TEST_CHECK(!RobotCanGuardCheck(NULL, 1u));
    TEST_CHECK(!RobotCanGuardCheck(items, 9u));
    TEST_CHECK(RobotCanGuardCheck(NULL, 0u));
    return 0;
}
