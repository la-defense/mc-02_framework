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

int main(void)
{
    if (TestRecovery() != 0) return 1;
    return TestWrapAndInvalid();
}
