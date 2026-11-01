/** @file module_plan_test.c
 * @brief 通过正式启动接口检查依赖顺序、核心失败与辅助降级。
 */
#include "module_plan.h"
#include "test_check.h"
#include <stdlib.h>
typedef struct { uint32_t calls; uint32_t fail; } Fixture;
static uint8_t start(void *context, unsigned module)
{
    Fixture *fixture = context;
    fixture->calls |= ROBOT_MODULE_BIT(module);
    return (fixture->fail & ROBOT_MODULE_BIT(module)) == 0u;
}
int main(void)
{
    RobotModulePlan_t plan = {.enabled = 7u, .required = 1u};
    plan.dependencies[0] = 2u;
    RobotModuleStartup_t state = {0};
    Fixture fixture = {0};
    TEST_CHECK(RobotModulePlanValidate(&plan, NULL) == ROBOT_MODULE_OK);
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(state.started == 7u && fixture.calls == 7u);
    fixture = (Fixture){.fail = 4u};
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(state.failed == 4u && state.started == 3u && !state.core_failed);
    TEST_CHECK(RobotModuleStartupMarkFailure(&plan, &state, ROBOT_MODULE_SUPERCAP) ==
               ROBOT_MODULE_ARGUMENT);
    fixture = (Fixture){0};
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(RobotModuleStartupMarkFailure(&plan, &state, ROBOT_MODULE_COMMUNICATION) ==
               ROBOT_MODULE_OK);
    TEST_CHECK((state.failed & ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION)) != 0u &&
               !state.core_failed);
    TEST_CHECK(RobotModuleStartupMarkFailure(&plan, &state, ROBOT_MODULE_IMU) ==
               ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(state.core_failed && state.failed_module == ROBOT_MODULE_IMU &&
               (state.blocked & ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY)) != 0u);
    fixture = (Fixture){.fail = 2u};
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(state.core_failed && (fixture.calls & 1u) == 0u);
    plan.dependencies[2] = 1u;
    plan.required = 0u;
    fixture = (Fixture){.fail = 1u};
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(state.blocked == 4u && (fixture.calls & 4u) == 0u);
    plan.enabled = 3u;
    fixture = (Fixture){0};
    TEST_CHECK(RobotModuleStart(&plan, start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(fixture.calls == 3u);
    plan.dependencies[0] = 4u;
    TEST_CHECK(RobotModulePlanValidate(&plan, NULL) == ROBOT_MODULE_DEPENDENCY);
    plan.dependencies[0] = 2u;
    plan.dependencies[1] = 1u;
    TEST_CHECK(RobotModulePlanValidate(&plan, NULL) == ROBOT_MODULE_CYCLE);
    plan.dependencies[1] = 0u;
    plan.required = 8u;
    TEST_CHECK(RobotModulePlanValidate(&plan, NULL) == ROBOT_MODULE_ARGUMENT);
    TEST_CHECK(RobotModuleStart(NULL, start, &fixture, &state) == ROBOT_MODULE_ARGUMENT);
    return EXIT_SUCCESS;
}
