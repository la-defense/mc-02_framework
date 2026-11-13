/** @file robot_can_requirements_test.c
 * @brief 检查实际车型字段到经典 CAN 端点需求的映射。
 */
#include "robot_can_requirements.h"
#include "test_check.h"

int main(void)
{
    Robot_Config_s config = {0}; // 测试配置只用于验证端点数量和总线映射。
    config.can_bridge.can_bus = PLATFORM_CAN_BUS_3; // 板间桥接。
    config.chassis_param.chassis_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 四轮电机。
    config.gimbal_param.yaw_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_2; // 大 yaw。
    config.gimbal_param.pitch_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_3; // pitch。
    config.shoot_param.friction_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 摩擦轮。
    config.shoot_param.loader_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_2; // 拨弹盘。
    config.shoot_param.shoot_type = FOUR_SHOOT; // 四个摩擦轮分别占端点。
    RobotCanFilterDemand_t demands[8] = {0}; // 固定容量由调用者提供。
    TEST_CHECK(RobotCollectCanDemands(&config, demands) == 7u);
    TEST_CHECK(demands[0].module == ROBOT_MODULE_COMMUNICATION && demands[0].bus == PLATFORM_CAN_BUS_3 && demands[0].standard == 1u);
    TEST_CHECK(demands[1].module == ROBOT_MODULE_CHASSIS && demands[1].bus == PLATFORM_CAN_BUS_1 && demands[1].standard == 4u);
    TEST_CHECK(demands[2].module == ROBOT_MODULE_CHASSIS && demands[2].bus == PLATFORM_CAN_BUS_2 && demands[2].standard == 1u);
    TEST_CHECK(demands[3].module == ROBOT_MODULE_GIMBAL && demands[3].bus == PLATFORM_CAN_BUS_2 && demands[3].standard == 1u);
    TEST_CHECK(demands[4].module == ROBOT_MODULE_GIMBAL && demands[4].bus == PLATFORM_CAN_BUS_3 && demands[4].standard == 1u);
    TEST_CHECK(demands[5].module == ROBOT_MODULE_SHOOTER && demands[5].bus == PLATFORM_CAN_BUS_1 && demands[5].standard == 4u);
    TEST_CHECK(demands[6].module == ROBOT_MODULE_SHOOTER && demands[6].bus == PLATFORM_CAN_BUS_2 && demands[6].standard == 1u);
    for (uint8_t i = 0u; i < 7u; ++i)
        TEST_CHECK(demands[i].extended == 0u); // 当前业务设备均使用标准 ID。
    const uint32_t chassis = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS); // 底盘作为核心模块。
    const uint32_t gimbal = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL); // 云台作为核心模块。
    const uint32_t bridge = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION); // 板间通信作为核心模块。
    const uint32_t shooter = ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER); // 本例发射机构为辅助模块。
    RobotModulePlan_t plan = {.enabled = chassis | gimbal | bridge | shooter, .required = chassis | gimbal | bridge};
    RobotCanFilterCapacity_t capacity = {.standard = {7u, 3u, 2u}, .endpoints = 13u}; // 总线一不足以容纳四个摩擦轮。
    RobotCanCapacityResult_t result = {0}; // 使用生产规划器验证整组需求。
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 7u, &capacity, &result) == ROBOT_MODULE_OK);
    TEST_CHECK(result.rejected_modules == shooter); // 辅助发射机构整体拒绝，核心仍获得容量。
    capacity.standard[0] = 3u; // 底盘的四个标准端点也无法容纳。
    TEST_CHECK(RobotCanCapacityPlan(&plan, demands, 7u, &capacity, &result) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(result.failed_core_module == ROBOT_MODULE_CHASSIS); // 精确报告第一个核心失败模块。
    config.shoot_param.shoot_type = DUAL_SHOOT; // 改变机构后摩擦轮数量应同步变化。
    TEST_CHECK(RobotCollectCanDemands(&config, demands) == 7u && demands[5].standard == 2u);
    return 0;
}
