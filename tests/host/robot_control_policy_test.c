/** @file robot_control_policy_test.c
 * @brief 验证纯C三模式许可和来源选择；不模拟电机实际运动。
 */
#include "robot_control_policy.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

static RobotControlFacts_t Facts(void)
{
    const RobotControlFacts_t facts = {
        .mode = ROBOT_CONTROL_MANUAL, // 默认纯手动，视觉不能主动接管。
        .manual = {1000u, 100u, 1u}, // 人工来源可用100ms。
        .aim = {1000u, 20u, 1u}, // 瞄准来源单独使用20ms。
        .navigation = {1000u, 50u, 1u}, // 导航来源单独使用50ms。
        .match = {1000u, 100u, 1u}, // 测试比赛阶段使用100ms。
        .feedback_valid_functions = ROBOT_CONTROL_ALL, // 测试各功能必需反馈均有效。
        .core_ready = 1u, // 已通过全局核心检查。
        .manual_enabled = 1u, // 人工已使能。
        .match_running = 1u, // 测试阶段允许运行。
        .manual_fire_requested = 1u, // 同时给出请求以验证各模式的击发来源。
        .vision_fire_requested = 1u, // 纯手动和辅助模式不能因此取得自动击发。
    };
    return facts;
}

static int Modes(void)
{
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 1u, 0u};
    RobotControlFacts_t facts = Facts();
    RobotControlPermits_t output;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.functions == ROBOT_CONTROL_ALL && output.chassis_source == ROBOT_CONTROL_SOURCE_MANUAL);
    CHECK(output.gimbal_source == ROBOT_CONTROL_SOURCE_MANUAL && output.fire_source == ROBOT_CONTROL_SOURCE_MANUAL);
    facts.mode = ROBOT_CONTROL_ASSIST;
    CHECK(RobotControlEvaluate(&policy, &facts, 1020u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.gimbal_source == ROBOT_CONTROL_SOURCE_VISION && output.fire_source == ROBOT_CONTROL_SOURCE_MANUAL);
    facts.manual_fire_requested = 0u; // 视觉允许击发也不能代替辅助模式中的人工请求。
    CHECK(RobotControlEvaluate(&policy, &facts, 1020u, &output) == ROBOT_CONTROL_OK && !output.fire_allowed);
    CHECK(RobotControlEvaluate(&policy, &facts, 1021u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.gimbal_source == ROBOT_CONTROL_SOURCE_MANUAL && !output.fire_allowed);
    facts.manual.valid = 0u; // 辅助模式不能在人工来源失效后自行变成自主。
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    facts.mode = ROBOT_CONTROL_AUTONOMOUS;
    facts.manual_enabled = 0u; // 正式自主资格不依赖人工使能或遥控在线。
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.functions == ROBOT_CONTROL_ALL && output.chassis_source == ROBOT_CONTROL_SOURCE_NAVIGATION);
    CHECK(output.gimbal_source == ROBOT_CONTROL_SOURCE_VISION && output.fire_source == ROBOT_CONTROL_SOURCE_VISION);
    CHECK(RobotControlEvaluate(&policy, &facts, 1021u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.chassis_source == ROBOT_CONTROL_SOURCE_NAVIGATION && output.gimbal_source == ROBOT_CONTROL_SOURCE_LOCAL && !output.fire_allowed);
    CHECK(RobotControlEvaluate(&policy, &facts, 1051u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.chassis_source == ROBOT_CONTROL_SOURCE_LOCAL && output.gimbal_source == ROBOT_CONTROL_SOURCE_LOCAL && !output.fire_allowed);
    CHECK(RobotControlEvaluate(&policy, &facts, 1101u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    facts.navigation.source_time_ms = 1101u; // 导航新数据不能替裁判阶段续期。
    CHECK(RobotControlEvaluate(&policy, &facts, 1101u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    return 0;
}

static int InhibitAndFeedback(void)
{
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 1u, 0u};
    RobotControlFacts_t facts = Facts();
    RobotControlPermits_t output;
    for (unsigned mode = 0u; mode <= ROBOT_CONTROL_AUTONOMOUS; ++mode) {
        facts.mode = (RobotControlMode_t)mode; // 同一禁止事实必须覆盖三种模式。
        facts.core_ready = 0u;
        CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
        facts.core_ready = 1u; facts.immediate_inhibit = 1u;
        CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK && !output.fire_allowed && output.functions == 0u);
        facts.immediate_inhibit = 0u;
    }
    facts.blocked_functions = ROBOT_CONTROL_SHOOTER; // 卡弹等局部故障不撤销底盘与云台。
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL) && !output.fire_allowed);
    facts.blocked_functions = 0u;
    facts.feedback_valid_functions = ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_SHOOTER; // 云台核心反馈缺失时不允许击发。
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.gimbal_source == ROBOT_CONTROL_SOURCE_NONE && !output.fire_allowed);
    facts = Facts();
    facts.mode = ROBOT_CONTROL_AUTONOMOUS; facts.match = (RobotControlLease_t){UINT32_MAX - 4u, 10u, 1u};
    CHECK(RobotControlEvaluate(&policy, &facts, 5u, &output) == ROBOT_CONTROL_OK && output.functions != 0u);
    CHECK(RobotControlEvaluate(&policy, &facts, 6u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    return 0;
}

static int CommissioningAndErrors(void)
{
    RobotControlPolicy_t policy = {ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL, 1u, 0u, 1u};
    RobotControlFacts_t facts = Facts();
    RobotControlPermits_t output;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK);
    CHECK(output.functions == policy.enabled_functions && !output.fire_allowed);
    facts.mode = ROBOT_CONTROL_ASSIST; // 调试固件不能因新视觉帧切到辅助控制。
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    facts.mode = ROBOT_CONTROL_AUTONOMOUS;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_OK && output.functions == 0u);
    memset(&output, 0xa5, sizeof(output));
    unsigned char previous[sizeof(output)];
    memcpy(previous, &output, sizeof(output)); // 配置错误不得交付部分新许可。
    policy.enabled_functions |= ROBOT_CONTROL_SHOOTER;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_BAD_CONFIG);
    CHECK(memcmp(previous, &output, sizeof(output)) == 0);
    policy.enabled_functions = ROBOT_CONTROL_CHASSIS;
    facts.mode = (RobotControlMode_t)-1;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_BAD_FACTS);
    facts = Facts(); facts.aim.max_age_ms = 0u;
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, &output) == ROBOT_CONTROL_BAD_FACTS);
    CHECK(memcmp(previous, &output, sizeof(output)) == 0);
    CHECK(RobotControlEvaluate(NULL, &facts, 1000u, &output) == ROBOT_CONTROL_BAD_ARGUMENT);
    CHECK(RobotControlEvaluate(&policy, NULL, 1000u, &output) == ROBOT_CONTROL_BAD_ARGUMENT);
    CHECK(RobotControlEvaluate(&policy, &facts, 1000u, NULL) == ROBOT_CONTROL_BAD_ARGUMENT);
    return 0;
}

int main(void)
{
    CHECK(Modes() == 0);
    CHECK(InhibitAndFeedback() == 0);
    CHECK(CommissioningAndErrors() == 0);
    return 0;
}
