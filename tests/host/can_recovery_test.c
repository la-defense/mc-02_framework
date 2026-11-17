/** @file can_recovery_test.c
 * @brief 用生产恢复策略检查旧反馈、连续有效时间和新命令。
 */
#include "can_recovery.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "line %d: %s\n", __LINE__, #c); return 1; } } while (0)

static int TestRecovery(void)
{
    CanRecovery_t state = {0};
    const CanRecoveryConfig_t config = {3u, 10u}; // 两路反馈均需连续有效 10ms。
    CanRecoveryInput_t input = {.now_ms = 100u, .valid_feedback_mask = 3u,
        .feedback_sequence = {10u, 20u}, .command_sequence = 5u, .bus_ok = 1u, .permit_ok = 1u};
    CHECK(CanRecoveryInit(&state, &config));
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 11u; // 一路更新不足以恢复。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[1] = 21u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.now_ms = 109u;
    CHECK(!CanRecoveryStep(&state, &input)); // 尚未满足连续时间。
    input.now_ms = 110u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.command_sequence = 4u;
    CHECK(!CanRecoveryStep(&state, &input)); // 旧序号不能冒充新命令。
    input.command_sequence = 6u;
    CHECK(CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_READY);
    input.command_sequence = 0u; // 来源报告命令过期时立即撤销。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.command_sequence = 6u;
    CHECK(!CanRecoveryStep(&state, &input)); // 同一旧命令重新出现也不能恢复。
    input.command_sequence = 7u;
    CHECK(CanRecoveryStep(&state, &input));
    input.bus_ok = 0u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_BUS);
    input.bus_ok = 1u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 12u;
    input.feedback_sequence[1] = 22u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.valid_feedback_mask = 1u; // 一路过期，中断计时并丢弃所有旧反馈。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.valid_feedback_mask = 3u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 13u;
    input.feedback_sequence[1] = 23u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.feedback_sequence[1] = 0u; // 有效位误置时，缺失序号仍不能继续计时。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 14u;
    input.feedback_sequence[1] = 24u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.permit_ok = 0u; // 外部许可撤销也要重新验证。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_BUS);
    return 0;
}

static int TestWrapAndInvalid(void)
{
    CanRecovery_t state = {0};
    CanRecovery_t before = state;
    CanRecoveryConfig_t config = {0u, 10u};
    CHECK(!CanRecoveryInit(&state, &config) && memcmp(&state, &before, sizeof state) == 0);
    config.required_feedback_mask = 1u;
    config.stable_ms = UINT32_MAX;
    CHECK(!CanRecoveryInit(&state, &config));
    config.stable_ms = 10u;
    CHECK(CanRecoveryInit(&state, &config));
    CanRecoveryInput_t input = {.now_ms = UINT32_MAX - 5u, .valid_feedback_mask = 1u,
        .feedback_sequence = {UINT32_MAX}, .command_sequence = UINT32_MAX,
        .bus_ok = 1u, .permit_ok = 1u};
    CHECK(!CanRecoveryStep(&state, &input));
    input.feedback_sequence[0] = 1u; // 序号跳过保留的零并回绕。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.now_ms = 4u; // 单调时间回绕后恰好经过 10ms。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.command_sequence = 1u;
    CHECK(CanRecoveryStep(&state, &input));
    ++input.inhibit_generation; // 两次读取之间发生并恢复的故障仍要撤销许可。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    CHECK(!CanRecoveryStep(NULL, &input) && !CanRecoveryStep(&state, NULL));
    return 0;
}

static int TestLostAndReplayedValues(void)
{
    CanRecovery_t state = {0};
    const CanRecoveryConfig_t config = {1u, 10u};
    CanRecoveryInput_t input = {.now_ms = 100u, .valid_feedback_mask = 1u,
        .feedback_sequence = {10u}, .command_sequence = 5u, .bus_ok = 1u, .permit_ok = 1u};
    CHECK(CanRecoveryInit(&state, &config));
    CHECK(!CanRecoveryStep(&state, &input));
    input.feedback_sequence[0] = 11u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.feedback_sequence[0] = 10u; // 收到旧帧时即使有效位误置也不能继续计时。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 11u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 12u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    input.command_sequence = 6u; // 恢复期间的新命令也必须成为禁止重放的基准。
    input.now_ms = 105u;
    CHECK(!CanRecoveryStep(&state, &input));
    input.command_sequence = 0u; // 到计时结束时来源可能已过期。
    input.now_ms = 110u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.command_sequence = 6u;
    CHECK(!CanRecoveryStep(&state, &input)); // 不能因为过期时填零就重新接受旧命令。
    input.command_sequence = 7u;
    CHECK(CanRecoveryStep(&state, &input));
    input.feedback_sequence[0] = 0u; // 缺少反馈时保留最后一次观察到的序号。
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 11u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 12u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.feedback_sequence[0] = 13u;
    CHECK(!CanRecoveryStep(&state, &input) && state.phase == CAN_RECOVERY_STABILIZING);
    return 0;
}

static int TestTimedSources(void)
{
    CanRecoveryTimed_t state = {0}; // 每条控制路径独立保存来源和恢复阶段。
    const CanRecoveryConfig_t config = {1u, 10u}; // 测试等待 10 ms。
    CanRecoveryTimedInput_t input = {
        .core = {.now_ms = 100u, .valid_feedback_mask = 1u, .feedback_sequence = {10u},
                 .command_sequence = 5u, .bus_ok = 1u, .permit_ok = 1u},
        .command_source_id = 1u, .command_source_ms = 100u, .command_max_age_ms = 100u};
    CHECK(CanRecoveryTimedInit(&state, &config));
    CHECK(!CanRecoveryTimedStep(&state, &input)); // 首次来源先记录已有数据。
    input.core.feedback_sequence[0] = 11u;
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_STABILIZING);
    input.core.now_ms = 110u;
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.core.now_ms = 111u; input.core.command_sequence = 6u; input.command_source_ms = 109u;
    CHECK(!CanRecoveryTimedStep(&state, &input)); // 稳定前产生、稳定后才交付的命令不能恢复。
    input.core.command_sequence = 7u; input.command_source_ms = 110u;
    CHECK(!CanRecoveryTimedStep(&state, &input)); // 与稳定完成同一毫秒也不判定为之后。
    input.core.command_sequence = 8u; input.command_source_ms = 111u;
    CHECK(CanRecoveryTimedStep(&state, &input)); // 新序号和新来源时间均满足。
    input.command_source_id = 2u; input.core.command_sequence = 1u; // 切换到另一个独立序号空间。
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.core.feedback_sequence[0] = 12u; input.core.now_ms = 112u; input.command_source_ms = 112u;
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_STABILIZING);
    input.core.now_ms = 122u; input.command_source_ms = 122u; input.core.command_sequence = 2u;
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.core.now_ms = 123u; input.command_source_ms = 123u; input.core.command_sequence = 3u;
    CHECK(CanRecoveryTimedStep(&state, &input));
    input.core.now_ms = 224u; // 同一份命令已超过 100 ms 期限。
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.core.now_ms = 225u; input.command_source_ms = 225u; input.core.command_sequence = 4u;
    CHECK(CanRecoveryTimedStep(&state, &input)); // 恢复后的新样本可以再次接管。
    ++input.command_source_epoch; // 两次读取之间切出又切回同一来源。
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    input.core.inhibit_generation++; // 两次调用之间有短故障。
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK);
    CHECK(CanRecoveryTimedInit(&state, &config)); // 使用独立实例检查时间与序号回绕。
    input = (CanRecoveryTimedInput_t){
        .core = {.now_ms = UINT32_MAX - 5u, .valid_feedback_mask = 1u, .feedback_sequence = {1u},
                 .command_sequence = UINT32_MAX, .bus_ok = 1u, .permit_ok = 1u},
        .command_source_id = 1u, .command_source_ms = UINT32_MAX - 5u, .command_max_age_ms = 100u};
    CHECK(!CanRecoveryTimedStep(&state, &input));
    input.core.feedback_sequence[0] = 2u;
    CHECK(!CanRecoveryTimedStep(&state, &input));
    input.core.now_ms = 4u; // 稳定时间和来源时刻跨过 32 位回绕。
    CHECK(!CanRecoveryTimedStep(&state, &input) && state.recovery.phase == CAN_RECOVERY_WAIT_COMMAND);
    input.core.command_sequence = 1u; input.command_source_ms = 4u;
    CHECK(!CanRecoveryTimedStep(&state, &input)); // 等于恢复边界仍不能接管。
    input.core.now_ms = 5u; input.command_source_ms = 5u; input.core.command_sequence = 2u;
    CHECK(CanRecoveryTimedStep(&state, &input)); // 回绕后的较新来源允许接管。
    return 0;
}

int main(void)
{
    if (TestRecovery() != 0) return 1;
    if (TestWrapAndInvalid() != 0) return 1;
    if (TestLostAndReplayedValues() != 0) return 1;
    return TestTimedSources();
}
