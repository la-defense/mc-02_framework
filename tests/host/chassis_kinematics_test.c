/** @file chassis_kinematics_test.c
 * @brief 用独立手算的轮速检查两种底盘；尺寸仅用于软件测试。
 */
#include "chassis_kinematics.h"
#include "mecanum.h"
#include "omni.h"
#include "test_check.h"
#include <math.h>
#include <stdlib.h>

static int close_to(float actual, float expected)
{
    return isfinite(actual) && fabsf(actual - expected) < 0.0001f;
}

static void TestOmniEstimate(void)
{
    OmniGeometry_t geometry = {.wheels = {
        {0.3f, 0.1f, 1.0f, 0.0f, 0.1f}, // 左前轮沿前方，转动投影为 -0.1 m。
        {0.1f, -0.4f, 0.0f, 1.0f, 0.2f}, // 右前轮沿左方，转动投影为 0.1 m。
        {-0.2f, 0.2f, -1.0f, 0.0f, 0.05f}, // 左后轮沿后方，转动投影为 0.2 m。
        {-0.3f, -0.4f, 0.0f, -1.0f, 0.25f}, // 右后轮沿右方，转动投影为 0.3 m。
    }};
    ChassisWheelSpeeds_t wheels = {{7.0f, -8.5f, -8.0f, 11.6f}}; // 手算 1 m/s、-2 m/s、3 rad/s 的轮轴转速。
    ChassisTwist_t result;
    TEST_CHECK(OmniEstimate(&wheels, &geometry, &result) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(result.vx_m_s, 1.0f) && close_to(result.vy_m_s, -2.0f) && close_to(result.wz_rad_s, 3.0f));

    ChassisVelocityEstimator_t estimator = {0};
    TEST_CHECK(ChassisKinematicsEstimatePrepared(&wheels, &estimator, &result) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(OmniPrepareEstimator(&geometry, &estimator) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(ChassisKinematicsEstimatePrepared(&wheels, &estimator, &result) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(result.vx_m_s, 1.0f) && close_to(result.vy_m_s, -2.0f) && close_to(result.wz_rad_s, 3.0f));
    const ChassisTwist_t reverse = {-2.0f, 0.5f, -1.0f}; // 同一组系数处理另一批不同方向的反馈。
    ChassisWheelSpeeds_t reverse_feedback;
    TEST_CHECK(OmniCalculate(&reverse, &geometry, &reverse_feedback) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(ChassisKinematicsEstimatePrepared(&reverse_feedback, &estimator, &result) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(result.vx_m_s, -2.0f) && close_to(result.vy_m_s, 0.5f) && close_to(result.wz_rad_s, -1.0f));

    wheels.rad_s[1] = NAN; // 无效轮速使整组输出清零。
    TEST_CHECK(ChassisKinematicsEstimatePrepared(&wheels, &estimator, &result) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(result.vx_m_s == 0.0f && result.vy_m_s == 0.0f && result.wz_rad_s == 0.0f);
    TEST_CHECK(OmniEstimate(&wheels, &geometry, &result) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(result.vx_m_s == 0.0f && result.vy_m_s == 0.0f && result.wz_rad_s == 0.0f);
    wheels.rad_s[1] = -8.5f;
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        geometry.wheels[i].drive_x = 1.0f; // 所有轮方向平行，无法推断横移。
        geometry.wheels[i].drive_y = 0.0f;
    }
    TEST_CHECK(OmniPrepareEstimator(&geometry, &estimator) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(ChassisKinematicsEstimatePrepared(&wheels, &estimator, &result) == CHASSIS_KINEMATICS_ARGUMENT); // 重配失败时旧系数不可用。
    TEST_CHECK(OmniEstimate(&wheels, &geometry, &result) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(OmniEstimate(NULL, &geometry, &result) == CHASSIS_KINEMATICS_ARGUMENT);
}

static void TestLocalPoseIntegration(void)
{
    const float pi = 3.14159265359f;
    ChassisLocalPose_t pose = {0};
    const ChassisTwist_t circle = {pi * 0.5f, 0.0f, pi * 0.5f}; // 半径 1 m，1 s 左转四分之一圈。
    TEST_CHECK(ChassisKinematicsIntegrate(&pose, &circle, &circle, 1.0f) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(pose.x_m, 1.0f) && close_to(pose.y_m, 1.0f) && close_to(pose.yaw_rad, pi * 0.5f));
    const ChassisTwist_t standstill = {0};
    const ChassisTwist_t forward = {2.0f, 0.0f, 0.0f}; // 平均 1 m/s，朝本地坐标的左向前进。
    TEST_CHECK(ChassisKinematicsIntegrate(&pose, &standstill, &forward, 1.0f) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(pose.x_m, 1.0f) && close_to(pose.y_m, 2.0f));
    const ChassisTwist_t turn = {0.0f, 0.0f, pi}; // 原地再左转半圈，输出角度回到 -π/2。
    TEST_CHECK(ChassisKinematicsIntegrate(&pose, &turn, &turn, 1.0f) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(pose.x_m, 1.0f) && close_to(pose.y_m, 2.0f) && close_to(pose.yaw_rad, -pi * 0.5f));
    const ChassisTwist_t invalid = {0.0f, 0.0f, NAN};
    TEST_CHECK(ChassisKinematicsIntegrate(&pose, &turn, &invalid, 1.0f) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(ChassisKinematicsIntegrate(&pose, &turn, &turn, 0.0f) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(close_to(pose.x_m, 1.0f) && close_to(pose.y_m, 2.0f) && close_to(pose.yaw_rad, -pi * 0.5f));
}

int main(void)
{
    ChassisTwist_t input = {1.0f, 0.0f, 0.0f};
    ChassisWheelSpeeds_t output;
    MecanumGeometry_t mecanum = {0.2f, 0.3f, 0.0f, 0.0f, 0.1f};
    TEST_CHECK(MecanumCalculate(&input, &mecanum, &output) == CHASSIS_KINEMATICS_OK);
    for (unsigned i = 0; i < 4u; ++i) TEST_CHECK(close_to(output.rad_s[i], 10.0f));
    input = (ChassisTwist_t){0.0f, 1.0f, 0.0f};
    TEST_CHECK(MecanumCalculate(&input, &mecanum, &output) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(output.rad_s[0], -10.0f) && close_to(output.rad_s[1], 10.0f));
    TEST_CHECK(close_to(output.rad_s[2], 10.0f) && close_to(output.rad_s[3], -10.0f));
    input = (ChassisTwist_t){0.0f, 0.0f, 1.0f};
    TEST_CHECK(MecanumCalculate(&input, &mecanum, &output) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(output.rad_s[0], -5.0f) && close_to(output.rad_s[1], 5.0f));
    TEST_CHECK(close_to(output.rad_s[2], -5.0f) && close_to(output.rad_s[3], 5.0f));

    // 四个测试轮沿正方形切向滚动，统一轮序为左前、右前、左后、右后。
    const float u = 0.70710678118f;
    OmniGeometry_t omni = {.wheels = {
        {0.2f, 0.2f, -u, u, 0.1f}, {0.2f, -0.2f, u, u, 0.1f},
        {-0.2f, 0.2f, -u, -u, 0.1f}, {-0.2f, -0.2f, u, -u, 0.1f}}};
    input = (ChassisTwist_t){1.0f, 0.0f, 0.0f};
    TEST_CHECK(OmniCalculate(&input, &omni, &output) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(output.rad_s[0], -7.0710678f) && close_to(output.rad_s[1], 7.0710678f));
    input = (ChassisTwist_t){0.0f, 1.0f, 0.0f};
    TEST_CHECK(OmniCalculate(&input, &omni, &output) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(output.rad_s[0], 7.0710678f) && close_to(output.rad_s[2], -7.0710678f));
    input = (ChassisTwist_t){0.0f, 0.0f, 1.0f};
    TEST_CHECK(OmniCalculate(&input, &omni, &output) == CHASSIS_KINEMATICS_OK);
    for (unsigned i = 0; i < 4u; ++i) TEST_CHECK(close_to(output.rad_s[i], 2.8284271f));
    input = (ChassisTwist_t){1.0f, 2.0f, 3.0f};
    TEST_CHECK(OmniCalculate(&input, &omni, &output) == CHASSIS_KINEMATICS_OK);
    TEST_CHECK(close_to(output.rad_s[0], 15.556349f));
    omni.wheels[0].radius_m = 0.0f;
    TEST_CHECK(OmniCalculate(&input, &omni, &output) == CHASSIS_KINEMATICS_ARGUMENT);
    for (unsigned i = 0; i < 4u; ++i) TEST_CHECK(output.rad_s[i] == 0.0f);
    mecanum.radius_m = NAN;
    TEST_CHECK(MecanumCalculate(&input, &mecanum, &output) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(MecanumCalculate(NULL, &mecanum, &output) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(OmniCalculate(&input, NULL, &output) == CHASSIS_KINEMATICS_ARGUMENT);
    TEST_CHECK(OmniCalculate(&input, &omni, NULL) == CHASSIS_KINEMATICS_ARGUMENT);
    TestOmniEstimate();
    TestLocalPoseIntegration();
    return EXIT_SUCCESS;
}
