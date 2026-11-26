/** @file robot_def_chassis_config_test.c
 * @brief 验证车型启动只读取对应的轮组几何，并拒绝非法设备参数。
 */
#include "config.h"
#include "test_check.h"
#if MC02_CHASSIS_IS_OMNI
#include "omni.h"
#else
#include "mecanum.h"
#endif

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static Robot_Config_s test_config;

const Robot_Config_s *SentinelConfigInit(void)
{
    return &test_config;
}

RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output);
const Chassis_Config_s *ChassisConfigFeed(void);

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
    chassis->chassis_motor_config.motor_type = M3508;
    chassis->chassis_motor_config.dji_chassis_output_limit_raw = 60u; // 仅用于主机车型测试的原始命令上限。
    for (unsigned index = 0u; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        chassis->wheel_motors[index].device_id = (uint8_t)(index + 1u); // 依轮序配置互不重复的测试编号。
        chassis->wheel_motors[index].direction = index % 2u == 0u ?
            MOTOR_AXIS_DIRECTION_NORMAL : MOTOR_AXIS_DIRECTION_REVERSE; // 允许左右轮采用不同安装方向。
    }

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

static void check_velocity_feedback_range(uint8_t startup)
{
    const Robot_Config_s *selected = NULL;
    set_valid_config();
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);

    ChassisVelocityEstimator_t estimator;
#if MC02_CHASSIS_IS_OMNI
    OmniGeometry_t *geometry = &test_config.chassis_param.omni_geometry; // 使用全部四轮的实际估速入口。
    for (unsigned wheel = 0u; wheel < CHASSIS_WHEEL_COUNT; ++wheel)
        geometry->wheels[wheel].radius_m = FLT_MAX; // 主机故障注入，轮径有限但超出可估速的范围。
    TEST_CHECK(OmniPrepareEstimator(geometry, &estimator) == CHASSIS_KINEMATICS_OK);
#else
    MecanumGeometry_t *geometry = &test_config.chassis_param.mecanum_geometry; // 复用麦轮本身的估速入口。
    geometry->radius_m = FLT_MAX; // 主机故障注入，轮径有限但超出可估速的范围。
    TEST_CHECK(MecanumPrepareEstimator(geometry, &estimator) == CHASSIS_KINEMATICS_OK);
#endif
    if (startup)
    {
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS); // 配置选择在启动设备前拒绝。
        TEST_CHECK(selected == NULL && ChassisConfigFeed() == NULL); // 失败时没有可用的旧车型选择。
    }
    else
    {
        const ChassisWheelSpeeds_t spin = {.rad_s = {-1.0f, 1.0f, -1.0f, 1.0f}}; // 四轮角速度有限，单位 rad/s。
        ChassisTwist_t feedback = {.vx_m_s = 10.0f, .vy_m_s = 20.0f, .wz_rad_s = 30.0f}; // 模拟查询前仍保存旧结果。
        TEST_CHECK(ChassisKinematicsEstimatePrepared(&spin, &estimator, &feedback) == CHASSIS_KINEMATICS_ARGUMENT);
        TEST_CHECK(feedback.vx_m_s == 0.0f && feedback.vy_m_s == 0.0f && feedback.wz_rad_s == 0.0f); // 任一分量溢出便清掉整组。
    }

    set_valid_config(); // 修正几何后依然允许正常选择。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
}

int main(int argc, char **argv)
{
    if (argc == 2)
    {
        const uint8_t startup = (uint8_t)(strcmp(argv[1], "feedback-range-startup") == 0); // 分别执行启动和运行期检查。
        TEST_CHECK(startup || strcmp(argv[1], "feedback-range-runtime") == 0);
        check_velocity_feedback_range(startup);
        return EXIT_SUCCESS;
    }
    const Robot_Config_s *selected = NULL;
    set_valid_config();
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK);
    TEST_CHECK(selected == &test_config);

    test_config.chassis_param.chassis_motor_config.dji_chassis_output_limit_raw = 0u; // 未确认上限不能选择运动配置。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    test_config.chassis_param.chassis_motor_config.dji_chassis_output_limit_raw = DJI_OUTPUT_MAX_RAW + 1u; // 超出驱动范围也拒绝。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config(); // 为后续轮组测试恢复有效参数。

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

    set_valid_config();
    test_config.chassis_param.chassis_motor_config.output_axis.gear_ratio = FLT_MAX; // 正值本身有限，轮轴 rad/s 转电机轴 deg/s 会溢出。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.chassis_motor_config.output_axis.gear_ratio = FLT_MIN; // 换回轮轴的系数有限，但最大速度乘以该系数会溢出。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.motion_limits.max_motor_speed_deg_s = FLT_TRUE_MIN; // 名义转速为正，换成轮轴 rad/s 后会被舍入为零。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.motion_limits.max_wz_deg_s = FLT_TRUE_MIN; // 底盘角速度从 deg/s 换成 rad/s 后会被舍入为零。
    test_config.chassis_param.motion_limits.rotate_wz_deg_s = 0.0f; // 旋转档设置有效，只检查总角速度上限的换算。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.chassis_motor_config.output_axis.gear_ratio = 1e36f; // 转换系数有限，乘上实际投影的轮速会溢出。
    test_config.chassis_param.motion_limits.max_wz_deg_s = 3600.0f; // 触发软件限速前的电机轴换算。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
#if MC02_CHASSIS_IS_OMNI
    test_config.chassis_param.omni_geometry.wheels[0].radius_m = FLT_MIN; // 零速度时几何可解，最大转速时该轮的速度会溢出。
#else
    test_config.chassis_param.mecanum_geometry.radius_m = FLT_MIN; // 相同的极小轮半径也会使麦轮原地转向溢出。
#endif
    test_config.chassis_param.motion_limits.max_wz_deg_s = 3600.0f; // 仅用主机边界值触发溢出。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS &&
               selected == NULL && ChassisConfigFeed() == NULL);
    set_valid_config();
    test_config.chassis_param.motion_limits.max_motor_speed_deg_s = 1.0f; // 允许用正常限速压低可计算的旋转目标。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);

    set_valid_config();
    test_config.chassis_param.wheel_motors[2].direction = (Motor_Axis_Direction_e)0; // 左后轮安装方向待确认。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.wheel_motors[3].device_id = 0u; // 右后轮没有有效设备编号。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    set_valid_config();
    test_config.chassis_param.wheel_motors[3].device_id = test_config.chassis_param.wheel_motors[0].device_id; // 右后轮与左前轮争用同一总线编号。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);

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
