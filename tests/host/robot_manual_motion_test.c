/** @file robot_manual_motion_test.c
 * @brief 检查人工 SI 意图到旧应用单位、参考方向和实际周期的转换。
 */
#include "robot_manual_motion.h"
#include "test_check.h"
#include <float.h>
#include <math.h>
#include <string.h>

int main(void)
{
    StdCmdManualIntent_t intent = {.source_time_ms = 100u, .sequence = 7u, .max_age_ms = 50u,
        .source = STD_CMD_REMOTE, .frame = STD_CMD_FRAME_CHASSIS}; // 隔离的测试输入。
    intent.axis[STD_CMD_VX] = 1.0f; intent.axis[STD_CMD_VY] = 0.5f;
    intent.axis[STD_CMD_WZ] = 1.0f; intent.axis[STD_CMD_YAW_RATE] = 2.0f;
    intent.axis[STD_CMD_PITCH_RATE] = -1.0f;
    RobotManualMotion_t output = {0};
    TEST_CHECK(RobotManualMotionProject(&intent, 110u, 10u, 20u, NAN, 30.0f, 5.0f, &output));
    TEST_CHECK(output.vx_mm_s == 1000.0f && output.vy_right_mm_s == -500.0f && output.offset_deg == 0.0f);
    TEST_CHECK(fabsf(output.wz_deg_s - 57.29578f) < 0.001f &&
        fabsf(output.yaw_deg - 31.145916f) < 0.001f && fabsf(output.pitch_deg - 4.427042f) < 0.001f);
    intent.frame = STD_CMD_FRAME_BIG_YAW;
    TEST_CHECK(RobotManualMotionProject(&intent, 110u, 0u, 20u, 25.0f, 30.0f, 5.0f, &output));
    TEST_CHECK(output.offset_deg == 25.0f && output.yaw_deg == 30.0f && output.pitch_deg == 5.0f);
    const RobotManualMotion_t prior = output; // 后续失败不得破坏上次完整结果。
    TEST_CHECK(!RobotManualMotionProject(&intent, 151u, 1u, 20u, 25.0f, 30.0f, 5.0f, &output));
    TEST_CHECK(!RobotManualMotionProject(&intent, 110u, 21u, 20u, 25.0f, 30.0f, 5.0f, &output));
    TEST_CHECK(!RobotManualMotionProject(&intent, 110u, 1u, 20u, NAN, 30.0f, 5.0f, &output));
    intent.frame = STD_CMD_FRAME_FIELD;
    TEST_CHECK(!RobotManualMotionProject(&intent, 110u, 1u, 20u, 0.0f, 30.0f, 5.0f, &output));
    TEST_CHECK(memcmp(&output, &prior, sizeof(output)) == 0); // 来源、周期和参照失败都不交付部分数据。
    intent.frame = STD_CMD_FRAME_CHASSIS; intent.axis[STD_CMD_VX] = FLT_MAX;
    TEST_CHECK(!RobotManualMotionProject(&intent, 110u, 1u, 20u, 0.0f, 30.0f, 5.0f, &output));
    intent.axis[STD_CMD_VX] = 0.0f; intent.source_time_ms = UINT32_MAX - 4u;
    TEST_CHECK(RobotManualMotionProject(&intent, 5u, 1u, 20u, 0.0f, 30.0f, 5.0f, &output)); // 计时回绕仍检查实际年龄。
    return 0;
}
