/** @file robot_can_requirements_disabled_test.c
 * @brief 验证双板构建中未启用通信时不会为桥接预留 CAN 端点。
 */
#include "robot_can_requirements.h"
#include "test_check.h"

int main(void)
{
    Robot_Config_s config = {0}; // 空配置不会被未启用模块读取。
    RobotCanFilterDemand_t demands[8] = {0}; // 固定容量与生产入口一致。
    TEST_CHECK(RobotCollectCanDemands(&config, demands) == 0u); // 本测试配置没有启用任何 CAN 用户。
    return 0;
}
