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
    return EXIT_SUCCESS;
}
