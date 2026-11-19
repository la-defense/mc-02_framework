/** @file robot_def_chassis_config_test.c
 * @brief 验证车型启动只读取对应的轮组几何，并拒绝非法设备参数。
 */
#include "config.h"
#include "test_check.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static Robot_Config_s test_config;

const Robot_Config_s *SentinelConfigInit(void)
{
    return &test_config;
}

RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output);

static void set_valid_config(void)
{
    memset(&test_config, 0, sizeof(test_config));
    Chassis_Config_s *chassis = &test_config.chassis_param;
    chassis->motion_limits = (ChassisMotionLimits_s){
        .configured = 1u,
        .max_wz_deg_s = 180.0f,
        .rotate_wz_deg_s = 90.0f,
        .max_motor_speed_deg_s = 720.0f,
    };
    chassis->chassis_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1;
    chassis->chassis_motor_config.output_axis.gear_ratio = 19.0f;
    chassis->chassis_motor_config.output_axis.direction = MOTOR_AXIS_DIRECTION_NORMAL;
    chassis->chassis_motor_config.motor_type = M3508;
    for (unsigned index = 0u; index < 4u; ++index)
        chassis->chassis_motor_id[index] = (uint8_t)(index + 1u);

#if MC02_CHASSIS_IS_OMNI
    chassis->wheel_type = OMNI_WHEEL; // 麦轮的几何成员保持为零。
    const float diagonal = 0.70710678118f;
    chassis->omni_geometry.wheels[0] = (ChassisWheelGeometry_t){0.2f, 0.2f, diagonal, -diagonal, 0.08f};
    chassis->omni_geometry.wheels[1] = (ChassisWheelGeometry_t){0.2f, -0.2f, diagonal, diagonal, 0.08f};
    chassis->omni_geometry.wheels[2] = (ChassisWheelGeometry_t){-0.2f, 0.2f, diagonal, diagonal, 0.08f};
    chassis->omni_geometry.wheels[3] = (ChassisWheelGeometry_t){-0.2f, -0.2f, diagonal, -diagonal, 0.08f};
#else
    chassis->wheel_type = MECANUM_WHEEL; // 全向轮的几何成员保持为零。
    chassis->mecanum_geometry = (MecanumGeometry_t){
        .half_base_m = 0.2f, // 测试半轴距，m。
        .half_track_m = 0.2f, // 测试半轮距，m。
        .center_x_m = 0.0f, // 计算中心在几何中心。
        .center_y_m = 0.0f, // 无横向偏移。
        .radius_m = 0.08f, // 测试轮半径，m。
    };
#endif
}

int main(void)
{
    const Robot_Config_s *selected = NULL;
    set_valid_config();
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK);
    TEST_CHECK(selected == &test_config);

#if MC02_CHASSIS_IS_OMNI
    test_config.chassis_param.odometry_max_gap_ms = 5u; // 合法的全向轮测试配置。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK);
    test_config.chassis_param.odometry_max_gap_ms = UINT32_C(0x80000000); // 超过 32 位毫秒时间可区分的先后范围。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);
    set_valid_config();
#endif

    test_config.chassis_param.chassis_motor_config.can_init_config.can_bus = PLATFORM_CAN_INVALID; // 未声明总线不可启动核心底盘。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL); // 错误配置不能沿用前一次选择结果。

    set_valid_config();
    test_config.chassis_param.chassis_motor_config.output_axis.gear_ratio = 0.0f; // 缺少唯一的减速比来源则拒绝。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);

    set_valid_config(); // 后续场景独立验证几何错误。
#if MC02_CHASSIS_IS_OMNI
    test_config.chassis_param.omni_geometry.wheels[0].drive_x = 2.0f;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);
    set_valid_config();
    for (unsigned index = 0u; index < 4u; ++index)
    {
        test_config.chassis_param.omni_geometry.wheels[index].drive_x = 0.0f; // 四个方向平行时无法横移。
        test_config.chassis_param.omni_geometry.wheels[index].drive_y = 1.0f;
    }
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);
#else
    test_config.chassis_param.mecanum_geometry.radius_m = 0.0f; // 麦轮轮径未确认。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);
    set_valid_config();
    test_config.chassis_param.mecanum_geometry.center_y_m = NAN; // 非有限的左向偏移不能交给运动计算。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);
#endif

    set_valid_config();
    test_config.chassis_param.wheel_type = MC02_CHASSIS_IS_OMNI ? MECANUM_WHEEL : OMNI_WHEEL; // 所选几何必须匹配编入的模型。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);

    return EXIT_SUCCESS;
}
