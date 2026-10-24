#include "chassis_motion.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

static const ChassisMotionGeometry_s geometry = {
    .wheel_base_mm = 350.0f,
    .track_width_mm = 300.0f,
    .center_gimbal_offset_x_mm = 0.0f,
    .center_gimbal_offset_y_mm = 0.0f,
    .wheel_radius_mm = 60.0f,
    .reduction_ratio = 19.0f,
};

static const ChassisMotionLimits_s limits = {
    .configured = 1,
    .max_wz_deg_s = 10.0f,
    .rotate_wz_deg_s = 8.0f,
    .max_motor_speed_deg_s = 10000.0f,
};

static ChassisMotionInput_s input(ChassisMotionMode_e mode, float vx, float vy, float angle)
{
    ChassisMotionInput_s value = {vx, vy, 0.0f, angle, mode};
    return value;
}

static int rejected(ChassisMotionInput_s in, ChassisMotionGeometry_s geo,
                    ChassisMotionLimits_s lim)
{
    ChassisMotionOutput_s out = {.valid = 1, .wz_deg_s = 77.0f,
                                 .motor_speed_deg_s = {1.0f, 2.0f, 3.0f, 4.0f}};
    CHECK(!ChassisMotionCalculate(&in, &geo, &lim, &out));
    CHECK(!out.valid && out.wz_deg_s == 0.0f);
    for (unsigned i = 0; i < 4; ++i) CHECK(out.motor_speed_deg_s[i] == 0.0f);
    return 0;
}

static int near(float actual, float expected, float tolerance)
{
    return isfinite(actual) && fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    ChassisMotionOutput_s out;
    ChassisMotionInput_s in = input(CHASSIS_MOTION_MODE_FOLLOW, 0, 0, 0.5f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.valid && near(out.wz_deg_s, -0.375f, 1e-6f));
    in.offset_angle_deg = -0.5f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(near(out.wz_deg_s, 0.375f, 1e-6f));
    in.offset_angle_deg = 0.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == 0.0f);
    in.offset_angle_deg = 180.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == -limits.max_wz_deg_s);

    /* Fixed vectors for the current mecanum signs and motor shaft conversion. */
    const float mm_s_to_motor_deg_s = (float)(180.0 / 3.14159265358979323846) * 19.0f / 60.0f;
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 100.0f, 0.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(near(out.motor_speed_deg_s[0], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[1], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[2], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[3], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 100.0f, 25.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(near(out.motor_speed_deg_s[0], -125.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[1], -75.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[2], 75.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[3], 125.0f * mm_s_to_motor_deg_s, 0.02f));
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 0.0f, 100.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(near(out.motor_speed_deg_s[0], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[1], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[2], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[3], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 100.0f, 0.0f, 90.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(near(out.motor_speed_deg_s[0], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[1], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[2], -100.0f * mm_s_to_motor_deg_s, 0.02f));
    CHECK(near(out.motor_speed_deg_s[3], 100.0f * mm_s_to_motor_deg_s, 0.02f));
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 0.0f, 0.0f, 0.0f);
    in.requested_wz_deg_s = 4.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == 4.0f);
    in.requested_wz_deg_s = 40.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == limits.max_wz_deg_s);
    in.requested_wz_deg_s = -40.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == -limits.max_wz_deg_s);
    in = input(CHASSIS_MOTION_MODE_ROTATE, 0.0f, 0.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == limits.rotate_wz_deg_s);
    CHECK(out.motor_speed_deg_s[0] < 0 && out.motor_speed_deg_s[1] < 0 &&
          out.motor_speed_deg_s[2] < 0 && out.motor_speed_deg_s[3] < 0);
    const float expected_rotate = -limits.rotate_wz_deg_s * 325.0f *
                                  (3.14159265358979323846f / 180.0f) * mm_s_to_motor_deg_s;
    CHECK(near(out.motor_speed_deg_s[0], expected_rotate, 0.02f));

    /* Follow output clamps in both directions. */
    in = input(CHASSIS_MOTION_MODE_FOLLOW, 0, 0, 180.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == -10.0f);
    in.offset_angle_deg = -180.0f;
    CHECK(ChassisMotionCalculate(&in, &geometry, &limits, &out));
    CHECK(out.wz_deg_s == 10.0f);

    /* Common scaling preserves signs and ratios under the motor-axis limit. */
    ChassisMotionLimits_s tight = limits;
    tight.max_motor_speed_deg_s = 1000.0f;
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 1000.0f, 250.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &tight, &out));
    float largest = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(isfinite(out.motor_speed_deg_s[i]));
        CHECK(fabsf(out.motor_speed_deg_s[i]) <= tight.max_motor_speed_deg_s);
        if (fabsf(out.motor_speed_deg_s[i]) > largest) largest = fabsf(out.motor_speed_deg_s[i]);
    }
    CHECK(near(largest, tight.max_motor_speed_deg_s, 1e-4f));
    CHECK(near(out.motor_speed_deg_s[0] / out.motor_speed_deg_s[1],
               1250.0f / 750.0f, 1e-5f));
    in = input(CHASSIS_MOTION_MODE_ROTATE, 1000.0f, 250.0f, 0.0f);
    CHECK(ChassisMotionCalculate(&in, &geometry, &tight, &out));
    CHECK(fabsf(out.wz_deg_s) < fabsf(limits.rotate_wz_deg_s));

    /* Reject missing limits and malformed configuration/commands. */
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 0, 0, 0);
    ChassisMotionLimits_s bad_limits = limits; bad_limits.configured = 0;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.max_wz_deg_s = 0;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.max_wz_deg_s = -1;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.rotate_wz_deg_s = -11;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.max_motor_speed_deg_s = -1;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.max_motor_speed_deg_s = 0;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.rotate_wz_deg_s = 11;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.rotate_wz_deg_s = NAN;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    bad_limits = limits; bad_limits.max_wz_deg_s = INFINITY;
    CHECK(rejected(in, geometry, bad_limits) == 0);
    ChassisMotionGeometry_s bad_geometry = geometry; bad_geometry.wheel_base_mm = 0;
    CHECK(rejected(in, bad_geometry, limits) == 0);
    bad_geometry = geometry; bad_geometry.track_width_mm = -1;
    CHECK(rejected(in, bad_geometry, limits) == 0);
    bad_geometry = geometry; bad_geometry.wheel_radius_mm = NAN;
    CHECK(rejected(in, bad_geometry, limits) == 0);
    bad_geometry = geometry; bad_geometry.reduction_ratio = INFINITY;
    CHECK(rejected(in, bad_geometry, limits) == 0);
    bad_geometry = geometry; bad_geometry.reduction_ratio = 0;
    CHECK(rejected(in, bad_geometry, limits) == 0);
    bad_geometry = geometry; bad_geometry.center_gimbal_offset_x_mm = NAN;
    CHECK(rejected(in, bad_geometry, limits) == 0);

    in = input((ChassisMotionMode_e)99, 0, 0, 0);
    CHECK(rejected(in, geometry, limits) == 0);
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 0, 0, 180.01f);
    CHECK(rejected(in, geometry, limits) == 0);
    for (unsigned field = 0; field < 4; ++field)
    {
        for (unsigned nonfinite = 0; nonfinite < 2; ++nonfinite)
        {
            in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, 0, 0, 0);
            const float invalid = nonfinite ? INFINITY : NAN;
            if (field == 0) in.vx_mm_s = invalid;
            else if (field == 1) in.vy_mm_s = invalid;
            else if (field == 2) in.requested_wz_deg_s = invalid;
            else in.offset_angle_deg = invalid;
            CHECK(rejected(in, geometry, limits) == 0);
        }
    }
    in = input(CHASSIS_MOTION_MODE_NO_FOLLOW, FLT_MAX, FLT_MAX, 45.0f);
    CHECK(rejected(in, geometry, limits) == 0);

    CHECK(!ChassisMotionCalculate(NULL, &geometry, &limits, &out));
    CHECK(!out.valid && out.motor_speed_deg_s[0] == 0.0f);
    CHECK(!ChassisMotionCalculate(&in, NULL, &limits, &out));
    CHECK(!ChassisMotionCalculate(&in, &geometry, NULL, &out));
    CHECK(!ChassisMotionCalculate(&in, &geometry, &limits, NULL));
    puts("chassis motion checks passed");
    return 0;
}
