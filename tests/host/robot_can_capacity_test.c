/** @file robot_can_capacity_test.c
 * @brief 验证过滤器分配的核心优先、原子分组和容量边界。
 */
#include "robot_can_capacity.h"
#include "test_check.h"
#include <string.h>

int main(void)
{
    const uint32_t chassis = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS);
    const uint32_t gimbal = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL);
    RobotModulePlan_t plan = {.enabled = chassis | gimbal, .required = gimbal}; // 高编号模块作为核心。
    const RobotCanFilterDemand_t demands[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 2u, 0u}, // 辅助底盘消耗两个标准槽。
        {ROBOT_MODULE_GIMBAL, PLATFORM_CAN_BUS_1, 2u, 0u}, // 核心云台需要两个标准槽。
        {ROBOT_MODULE_GIMBAL, PLATFORM_CAN_BUS_2, 0u, 1u}, // 同一核心模块另需一个扩展槽。
    };
    RobotCanFilterCapacity_t capacity = {.standard = {3u, 0u, 0u}, .extended = {0u, 1u, 0u}, .endpoints = 4u};
    RobotCanCapacityResult_t result = {0};
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 3u, &capacity, &result) == ROBOT_MODULE_OK);
    TEST_CHECK(result.rejected_modules == chassis && result.failed_core_module == ROBOT_MODULE_COUNT); // 核心先于低编号辅助模块分配。
    capacity.extended[1] = 0u; // 第二条总线缺少核心扩展槽。
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 3u, &capacity, &result) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(result.failed_core_module == ROBOT_MODULE_GIMBAL); // 多总线需求整体失败。
    capacity.extended[1] = 1u; capacity.endpoints = 2u; // 两条总线虽有槽，但全局端点不足。
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 3u, &capacity, &result) == ROBOT_MODULE_CORE_FAILURE);
    capacity.endpoints = 5u; capacity.standard[0] = 4u;
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 3u, &capacity, &result) == ROBOT_MODULE_OK);
    TEST_CHECK(result.rejected_modules == 0u); // 两个模块都能完整分配。
    plan.dependencies[ROBOT_MODULE_GIMBAL] = chassis; // 核心间接依赖的底盘也属于核心。
    capacity.standard[0] = 3u;
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 3u, &capacity, &result) == ROBOT_MODULE_CORE_FAILURE);
    RobotCanCapacityResult_t previous = result; // 非法参数不能覆盖已有结果。
    TEST_CHECK(RobotCanCapacityPlan(&plan, NULL, 1u, &capacity, &result) == ROBOT_MODULE_ARGUMENT);
    TEST_CHECK(memcmp(&result, &previous, sizeof(result)) == 0);
    RobotCanFilterDemand_t invalid = demands[0];
    invalid.bus = PLATFORM_CAN_INVALID; // 未绑定的总线不能被当成第一条。
    TEST_CHECK(RobotCanCapacityPlan(&plan, &invalid, 1u, &capacity, &result) == ROBOT_MODULE_ARGUMENT);
    TEST_CHECK(memcmp(&result, &previous, sizeof(result)) == 0);
    invalid.bus = PLATFORM_CAN_BUS_1; invalid.module = ROBOT_MODULE_COUNT;
    TEST_CHECK(RobotCanCapacityPlan(&plan, &invalid, 1u, &capacity, &result) == ROBOT_MODULE_ARGUMENT);
    TEST_CHECK(memcmp(&result, &previous, sizeof(result)) == 0);
    memset(&plan, 0, sizeof(plan)); // 单独检查辅助依赖被拒绝时的容量复用。
    const uint32_t log = ROBOT_MODULE_BIT(ROBOT_MODULE_LOG);
    const uint32_t shooter = ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER);
    plan.enabled = chassis | gimbal | log | shooter; plan.required = gimbal;
    plan.dependencies[ROBOT_MODULE_CHASSIS] = log; // 低编号模块依赖高编号的辅助模块。
    const RobotCanFilterDemand_t auxiliary[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 1u, 0u}, // 上游失败后不能占槽。
        {ROBOT_MODULE_LOG, PLATFORM_CAN_BUS_1, 2u, 0u}, // 容量不足的辅助上游。
        {ROBOT_MODULE_SHOOTER, PLATFORM_CAN_BUS_1, 1u, 0u}, // 无关辅助模块仍可使用剩余槽。
    };
    capacity.standard[0] = 1u; capacity.endpoints = 1u;
    TEST_CHECK(RobotCanCapacityPlan(&plan, auxiliary, 3u, &capacity, &result) == ROBOT_MODULE_OK);
    TEST_CHECK((result.rejected_modules & (chassis | log)) == (chassis | log));
    TEST_CHECK((result.rejected_modules & shooter) == 0u);
    return 0;
}
