/** @file robot_control_feedback_test.c
 * @brief 检查车型电机和姿态期限如何共同影响各功能许可输入。
 */
#include "robot_control_feedback.h"
#include "message_center.h"
#include "test_check.h"
#include <string.h>

static DJI_RecoveryFeedbackItem_s Motor(MotorOutputRole_t role, Motor_Type_e type, uint32_t id, uint32_t time)
{
    return (DJI_RecoveryFeedbackItem_s){.bus = PLATFORM_CAN_BUS_1, .role = role, .motor_type = type,
        .motor_id = id, .sequence = 1u, .source_time_ms = time, .speed_valid = 1u,
        .current_valid = 1u, .position_valid = 1u}; // 数据仅用于主机测试。
}

int main(void)
{
    const RobotCanMotorPlan_t plan = {.count = 4u, .motors = {
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = 1u},
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = 2u},
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_GIMBAL, .type = GM6020, .id = 1u, .require_position = 1u},
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_LOADER, .type = M2006, .id = 3u}}}; // 只为测试声明的设备清单。
    RobotControlFeedback_t state = {0};
    TEST_CHECK(RobotControlFeedbackInit(&state, &plan));
    DJI_RecoveryFeedback_s actual = {.count = 4u};
    actual.motors[0] = Motor(MOTOR_OUTPUT_CHASSIS, M3508, 1u, 95u);
    actual.motors[1] = Motor(MOTOR_OUTPUT_CHASSIS, M3508, 2u, 97u);
    actual.motors[2] = Motor(MOTOR_OUTPUT_GIMBAL, GM6020, 1u, 99u);
    actual.motors[3] = Motor(MOTOR_OUTPUT_LOADER, M2006, 3u, 98u);
    MessageFreshness_t pose = {.valid = 1u, .source_clock = MESSAGE_CLOCK_LOCAL, .timestamp_ms = 60u};
    RobotControlFeedbackResult_t result = {0};
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == ROBOT_CONTROL_ALL && result.registered_motor_mask[0] == 3u &&
        result.registered_motor_mask[1] == 4u && result.registered_motor_mask[2] == 8u);
    TEST_CHECK(result.feedback[0].source_time_ms == 95u && result.feedback[0].max_age_ms == 20u);
    TEST_CHECK(result.feedback[1].source_time_ms == 99u && result.feedback[1].max_age_ms == 11u); // 姿态在 110 ms 先到期。
    TEST_CHECK(result.feedback[2].source_time_ms == 98u && result.feedback[2].max_age_ms == 20u);
    pose.timestamp_ms = 70u; // 姿态刷新，但云台电机仍是同一采样。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.feedback[1].source_time_ms == 99u && result.feedback[1].max_age_ms == 11u); // 同一电机采样不能被更新姿态续期。
    actual.motors[2].source_time_ms = 100u; actual.motors[2].sequence = 2u; // 该电机也提供了新采样。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.feedback[1].source_time_ms == 100u && result.feedback[1].max_age_ms == 20u); // 新样本可按实际期限重新判断。
    actual.motors[0].gap_generation++; // 两次读取之间发生断流，即使最新帧已有效也要报告。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER));
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == ROBOT_CONTROL_ALL); // 下轮交给安全管理执行恢复屏障。
    actual.motors[0].source_time_ms = 94u; // 单台设备倒退，其他轮的有效数据不能掩盖。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER));
    actual.motors[0].source_time_ms = 95u; // 恢复该电机上次接受的真实时刻。
    actual.motors[2].position_valid = 0u; // 内部位置参考不可信。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_SHOOTER));
    actual.motors[2].position_valid = 1u; pose.source_clock = MESSAGE_CLOCK_REMOTE_RAW;
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_SHOOTER)); // 未映射时钟不能授权云台。
    pose.source_clock = MESSAGE_CLOCK_LOCAL;
    actual.motors[3].current_valid = 0u; // 仅发射反馈失效。
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(result.valid_functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL));
    actual.motors[3].current_valid = 1u; actual.count = 5u; // 未声明用途的设备不能被忽略。
    actual.motors[4] = Motor(MOTOR_OUTPUT_UNASSIGNED, M3508, 4u, 100u);
    TEST_CHECK(RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result) && result.valid_functions == 0u);
    const RobotControlFeedbackResult_t prior = result; actual.count = DJI_MOTOR_CNT + 1u;
    TEST_CHECK(!RobotControlFeedbackCollect(&state, &actual, &pose, 100u, &result));
    TEST_CHECK(memcmp(&prior, &result, sizeof(result)) == 0); // 非法容量不交付部分结果。
    RobotCanMotorPlan_t duplicate = plan; duplicate.motors[1] = duplicate.motors[0];
    TEST_CHECK(!RobotControlFeedbackInit(&state, &duplicate)); // 重复身份不能覆盖已初始化实例。
    return 0;
}
