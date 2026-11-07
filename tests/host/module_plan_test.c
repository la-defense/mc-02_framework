/** @file module_plan_test.c
 * @brief 通过正式启动接口检查依赖顺序、核心失败与辅助降级。
 */
#include "module_plan.h"
#include "test_check.h"
#include <stdlib.h>
typedef struct { uint32_t calls; uint32_t fail; uint32_t at_barrier; uint32_t resource_failure; unsigned barriers; uint8_t barrier_ok; } Fixture;
static uint8_t start(void *context, unsigned module)
{
    Fixture *fixture = context;
    fixture->calls |= ROBOT_MODULE_BIT(module);
    return (fixture->fail & ROBOT_MODULE_BIT(module)) == 0u;
}
static uint8_t barrier(void *context, uint32_t *failed)
{
    Fixture *fixture = context;
    fixture->at_barrier = fixture->calls; // 保存阶段切换时已经启动的模块。
    ++fixture->barriers;
    *failed = fixture->resource_failure; // 模拟电源准备的具体失败范围。
    return fixture->barrier_ok;
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
    plan = (RobotModulePlan_t){.enabled = 15u, .required = 1u}; // 使用四个模块模拟两阶段依赖。
    plan.dependencies[0] = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 核心依赖需要延后。
    plan.dependencies[2] = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY); // 该下游也必须延后。
    fixture = (Fixture){.barrier_ok = 1u};
    TEST_CHECK(RobotModuleStartPhased(&plan, ROBOT_MODULE_BIT(ROBOT_MODULE_IMU), barrier,
                                      start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(fixture.barriers == 1u && fixture.at_barrier == 8u && fixture.calls == 15u);
    fixture = (Fixture){.barrier_ok = 1u, .resource_failure = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)};
    TEST_CHECK(RobotModuleStartPhased(&plan, ROBOT_MODULE_BIT(ROBOT_MODULE_IMU), barrier,
                                      start, &fixture, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(state.core_failed && state.failed_module == ROBOT_MODULE_IMU && fixture.calls == 8u);
    plan.required = 8u; // 将延后链设为辅助，资源失败只阻止其下游。
    fixture = (Fixture){.barrier_ok = 1u, .resource_failure = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)};
    TEST_CHECK(RobotModuleStartPhased(&plan, ROBOT_MODULE_BIT(ROBOT_MODULE_IMU), barrier,
                                      start, &fixture, &state) == ROBOT_MODULE_OK);
    TEST_CHECK(state.failed == 2u && state.blocked == 5u && fixture.calls == 8u);
    fixture = (Fixture){.barrier_ok = 1u, .fail = 8u}; // 第一阶段核心失败不能打开后续电源。
    TEST_CHECK(RobotModuleStartPhased(&plan, 2u, barrier, start, &fixture, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(fixture.barriers == 0u);
    fixture = (Fixture){.barrier_ok = 1u, .resource_failure = 8u}; // 阶段不能伪造其他模块失败。
    TEST_CHECK(RobotModuleStartPhased(&plan, 2u, barrier, start, &fixture, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(state.core_failed && fixture.barriers == 1u);
    fixture = (Fixture){0}; // 阶段回调自身失败时保持失能，不运行后续模块。
    TEST_CHECK(RobotModuleStartPhased(&plan, 2u, barrier, start, &fixture, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(state.core_failed && fixture.calls == 8u && fixture.barriers == 1u);
    fixture = (Fixture){.barrier_ok = 1u};
    TEST_CHECK(RobotModuleStartPhased(&plan, 16u, barrier, start, &fixture, &state) == ROBOT_MODULE_ARGUMENT);
    TEST_CHECK(fixture.calls == 0u && fixture.barriers == 0u); // 未启用模块不能进入阶段计划。
    return EXIT_SUCCESS;
}
