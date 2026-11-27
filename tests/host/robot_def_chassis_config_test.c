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
static unsigned source_reads; // 车型预检只查声明，不应读取实际反馈。

static Motor_Feedback_Status_e read_source(const void *context, Motor_Signal_Snapshot_s *output)
{
    (void)context;
    (void)output;
    ++source_reads;
    return MOTOR_FEEDBACK_NOT_READY; // 启动前来源还没有样本。
}

const Robot_Config_s *SentinelConfigInit(void)
{
    return &test_config;
}

RobotConfigStatus_t RobotConfigSelect(uint8_t id, const Robot_Config_s **output);
const Chassis_Config_s *ChassisConfigFeed(void);

static void set_valid_config(void)
{
    memset(&test_config, 0, sizeof(test_config));
    test_config.motor_periods.compute_us = 2000u; // 主机使用 2 ms 作为名义计算周期。
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
    chassis->chassis_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP; // 轮轴目标是 rad/s。
    chassis->chassis_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP; // 默认只启用速度环。
    chassis->chassis_motor_config.controller_param_init_config.speed_PID = (PID_Init_Config_s){
        .DtMin_s = MOTOR_PID_DT_MIN_S, // 测试允许电机的最小计算间隔。
        .DtMax_s = MOTOR_PID_DT_MAX_S, // 测试允许电机的最大计算间隔。
    };
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

static void check_wheel_control(uint8_t check_pid)
{
    set_valid_config();
    const Robot_Config_s *selected = NULL;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    Motor_Init_Config_s *motor = &test_config.chassis_param.chassis_motor_config;
    Motor_Control_Setting_s *setting = &motor->controller_setting_init_config;
    Motor_Controller_Init_s *pid = &motor->controller_param_init_config;
    if (check_pid)
    {
        pid->speed_PID.MaxOut = NAN; // 已启用速度环不能带非有限输出限值。
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        pid->speed_PID.MaxOut = 0.0f; // 零上限沿用 PID 原接口的有效取值。
        setting->close_loop_type = SPEED_AND_CURRENT_LOOP; // 启用串接的电流环。
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL); // 启用的电流环缺少有效计算间隔。
        pid->current_PID = pid->speed_PID; // 为测试电流环提供同样合法的时间参数。
        pid->current_PID.DtMax_s = 0.0f; // 最大间隔小于最小间隔。
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        pid->current_PID = pid->speed_PID; // 两个已启用的环都通过检查。
        pid->angle_PID.DtMax_s = NAN; // 未启用的位置环不参与底盘校验。
    }
    else
    {
        setting->outer_loop_type = OPEN_LOOP; // 开环会把速度数值当成设备命令。
        setting->close_loop_type = OPEN_LOOP;
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        setting->outer_loop_type = ANGLE_LOOP; // 位置外环会把同一数值当成 rad。
        setting->close_loop_type = ANGLE_AND_SPEED_LOOP;
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        setting->outer_loop_type = SPEED_LOOP;
        setting->close_loop_type = CURRENT_LOOP; // 声明速度外环却没有启用速度环。
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        setting->close_loop_type = (Closeloop_Type_e)(SPEED_LOOP | 0x80u); // 未识别的环选项不能默默通过。
        TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
        setting->close_loop_type = SPEED_LOOP; // 恢复速度环后重新选择。
    }
    TEST_CHECK(ChassisConfigFeed() == NULL); // 最近一次非法选择未保留底盘配置。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
}

static void check_wheel_compute_period(void)
{
    set_valid_config();
    const Robot_Config_s *selected = NULL;
    Motor_Init_Config_s *motor = &test_config.chassis_param.chassis_motor_config;
    Motor_Controller_Init_s *pid = &motor->controller_param_init_config;
    pid->speed_PID.DtMin_s = 0.004f; // 有效区间从 4 ms 开始，挡住了正常的 2 ms 拍点。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    TEST_CHECK(ChassisConfigFeed() == NULL);
    pid->speed_PID.DtMin_s = 0.002f; // 刚好等于下限可以选用，未启用的电流环可留空。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    motor->controller_setting_init_config.close_loop_type = SPEED_AND_CURRENT_LOOP;
    pid->current_PID = pid->speed_PID;
    pid->current_PID.DtMin_s = MOTOR_PID_DT_MIN_S;
    pid->current_PID.DtMax_s = 0.001f; // 电流环合法但最多接受 1 ms。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    pid->current_PID.DtMax_s = 0.002f; // 刚好等于电流环上限也可选用。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    test_config.motor_periods.compute_us = 0u; // 缺少计算周期时不交付车型。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
}

static void check_wheel_control_sources(void)
{
    set_valid_config();
    const Robot_Config_s *selected = NULL;
    Motor_Init_Config_s *motor = &test_config.chassis_param.chassis_motor_config;
    Motor_Control_Setting_s *setting = &motor->controller_setting_init_config;
    Motor_Controller_Init_s *controller = &motor->controller_param_init_config;
    setting->speed_feedback_source = OTHER_FEED; // 外部速度来源启用，但还没有读取回调。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    TEST_CHECK(ChassisConfigFeed() == NULL);
    controller->other_speed_feedback = (Motor_Signal_Source_s){
        .read = read_source, // 回调只在实际控制周期采样。
        .max_age_ms = 20u, // 测试样本期限，单位 ms。
        .unit = MOTOR_UNIT_RAD, // 刻意使用位置单位。
        .reference = MOTOR_REFERENCE_EXTERNAL, // 外部速度声明应使用外部参考系。
    };
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    controller->other_speed_feedback.unit = MOTOR_UNIT_RAD_S; // 修正后可启用外部速度反馈。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    setting->feedforward_flag = SPEED_FEEDFORWARD; // 启用速度前馈后，必须提供输出轴速度来源。
    controller->speed_feedforward = controller->other_speed_feedback; // 当前参考系错误，仍标成外部坐标。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    controller->speed_feedforward.reference = MOTOR_REFERENCE_OUTPUT_AXIS; // 前馈按轮轴方向表达。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    setting->feedforward_flag = CURRENT_FEEDFORWARD; // 电流前馈也需要自己的单位和来源。
    controller->current_feedforward = controller->speed_feedforward; // 误配成 rad/s 时不能启动。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    controller->current_feedforward.unit = MOTOR_UNIT_DEVICE_CURRENT; // DJI 使用设备电流单位。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &test_config);
    setting->motor_reverse_flag = MOTOR_DIRECTION_REVERSE; // 安装方向已经由各轮提供，不能再反转。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    TEST_CHECK(source_reads == 0u);
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "motor-sources") == 0)
    {
        check_wheel_control_sources();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strcmp(argv[1], "motor-period") == 0)
    {
        check_wheel_compute_period();
        return EXIT_SUCCESS;
    }
    if (argc == 2 && strncmp(argv[1], "motor-", 6u) == 0)
    {
        const uint8_t check_pid = (uint8_t)(strcmp(argv[1], "motor-pid") == 0); // PID 和环组合分别执行。
        TEST_CHECK(check_pid || strcmp(argv[1], "motor-mode") == 0);
        check_wheel_control(check_pid);
        return EXIT_SUCCESS;
    }
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
