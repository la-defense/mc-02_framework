/** @file robot_def_chassis_config_test.c
 * @brief 验证车型启动入口拒绝非法全向轮几何。
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
    chassis->wheel_type = OMNI_WHEEL;
    chassis->wheel_measure = (Wheel_Measure_s){
        .wheel_base = 400.0f,
        .track_width = 400.0f,
        .radius_wheel = 80.0f,
        .reduction_ratio_wheel = 19.0f,
    };
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

    const float diagonal = 0.70710678118f;
    chassis->omni_geometry.wheels[0] = (ChassisWheelGeometry_t){0.2f, 0.2f, diagonal, -diagonal, 0.08f};
    chassis->omni_geometry.wheels[1] = (ChassisWheelGeometry_t){0.2f, -0.2f, diagonal, diagonal, 0.08f};
    chassis->omni_geometry.wheels[2] = (ChassisWheelGeometry_t){-0.2f, 0.2f, diagonal, diagonal, 0.08f};
    chassis->omni_geometry.wheels[3] = (ChassisWheelGeometry_t){-0.2f, -0.2f, diagonal, -diagonal, 0.08f};
}

int main(void)
{
    const Robot_Config_s *selected = NULL;
    set_valid_config();
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK);
    TEST_CHECK(selected == &test_config);

    test_config.chassis_param.chassis_motor_config.can_init_config.can_bus = PLATFORM_CAN_INVALID; // 未声明总线不可启动核心底盘。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL); // 错误配置不能沿用前一次选择结果。

    set_valid_config(); // 后续场景独立验证几何错误。

    test_config.chassis_param.omni_geometry.wheels[0].drive_x = 2.0f;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);

    set_valid_config();
    for (unsigned index = 0u; index < 4u; ++index)
    {
        test_config.chassis_param.omni_geometry.wheels[index].drive_x = 0.0f;
        test_config.chassis_param.omni_geometry.wheels[index].drive_y = 1.0f;
    }
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS);
    TEST_CHECK(selected == NULL);

    return EXIT_SUCCESS;
}
