/** @file can_recovery.c
 * @brief 在固定内存中检查总线故障后的反馈与命令恢复顺序。
 */
#include "can_recovery.h"
#include <limits.h>
#include <stddef.h>

uint8_t CanRecoveryInit(CanRecovery_t *state, const CanRecoveryConfig_t *config)
{
    if (state == NULL || config == NULL || config->required_feedback_mask == 0u ||
        (config->required_feedback_mask & ~UINT32_C(0xffff)) != 0u ||
        config->stable_ms == 0u || config->stable_ms > INT32_MAX)
        return 0u; // 无效配置不能改变调用方的旧实例。
    CanRecovery_t candidate = {0}; // 新实例没有旧反馈或命令。
    candidate.config = *config; // 固定复制车型配置，避免运行中被改写。
    candidate.phase = CAN_RECOVERY_WAIT_BUS; // 启动时先禁止控制。
    candidate.initialized = 1u; // 完成所有设置后才标记可调用。
    *state = candidate; // 最后一次性交付完整实例。
    return 1u;
}

static void CaptureBaseline(CanRecovery_t *state, const CanRecoveryInput_t *input)
{
    for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
        state->baseline[index] = input->feedback_sequence[index]; // 保存禁止时可见的每路反馈。
    state->seen_mask = 0u; // 每次禁止后都要求重新收到反馈。
    state->command_baseline = input->command_sequence; // 同时放弃此前的命令。
    state->inhibit_generation = input->inhibit_generation; // 记录当前已见的禁止事件代际。
    state->baseline_captured = 1u; // 后续可与新的反馈比较。
}

static uint8_t SequenceIsNewer(uint32_t current, uint32_t previous)
{
    if (current == 0u) return 0u; // 零保留给尚无样本或命令。
    if (previous == 0u) return 1u; // 之前尚无数据时，首个非零序号有效。
    const uint32_t distance = current - previous; // 按无符号差值处理序号回绕。
    return (uint8_t)(distance != 0u && distance <= INT32_MAX); // 相同、倒退或无法判断的序号均不能接管。
}

uint8_t CanRecoveryStep(CanRecovery_t *state, const CanRecoveryInput_t *input)
{
    if (state == NULL || input == NULL || state->initialized != 1u)
        return 0u; // 无效输入不能授予许可。
    const uint32_t required = state->config.required_feedback_mask; // 本实例的全部必需反馈。
    if (input->bus_ok != 1u || input->permit_ok != 1u)
    {
        CaptureBaseline(state, input); // 故障持续期间反复记录当前基准，不能复用期间的旧数据。
        state->phase = CAN_RECOVERY_WAIT_BUS; // 总线或外部许可不可用时立即禁止。
        return 0u;
    }
    if (state->baseline_captured == 0u)
    {
        CaptureBaseline(state, input); // 首次运行也不能把启动前的数据视为新反馈。
        state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 后续等待各路反馈更新。
        return 0u;
    }
    if (state->inhibit_generation != input->inhibit_generation)
    {
        CaptureBaseline(state, input); // 两次调用之间的短故障也必须撤销原来的恢复结果。
        state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 即使总线已经恢复，也先等待新的反馈。
        return 0u;
    }
    uint32_t valid = input->valid_feedback_mask; // 从调用方提供的有效位开始检查。
    for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
        if (input->feedback_sequence[index] == 0u)
            valid &= ~(UINT32_C(1) << index); // 没有样本序号时，即使有效位置位也不能采信。
    if ((valid & required) != required)
    {
        CaptureBaseline(state, input); // 任一路过期都会中断连续有效计时。
        state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 失效后要求各路重新更新。
        return 0u;
    }
    if (state->phase == CAN_RECOVERY_WAIT_BUS)
        state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 总线恢复不直接授权运动。
    if (state->phase == CAN_RECOVERY_WAIT_FEEDBACK)
    {
        for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
        {
            const uint32_t bit = UINT32_C(1) << index; // 每一位对应同序号的必需反馈。
            if ((required & bit) != 0u && SequenceIsNewer(input->feedback_sequence[index], state->baseline[index]))
                state->seen_mask |= bit; // 只记录禁止后更新过的有效样本。
        }
        if ((state->seen_mask & required) != required)
            return 0u; // 尚有反馈没有更新，继续等待。
        state->stable_start_ms = input->now_ms; // 所有反馈已更新，从本时刻计时。
        state->phase = CAN_RECOVERY_STABILIZING; // 接下来仍须保持全部反馈有效。
        return 0u;
    }
    if (state->phase == CAN_RECOVERY_STABILIZING)
    {
        const uint32_t elapsed = input->now_ms - state->stable_start_ms; // 无符号减法支持计数回绕。
        if (elapsed > INT32_MAX)
        {
            CaptureBaseline(state, input); // 时间倒退或超出可判断范围时放弃旧等待。
            state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 重新验证每路反馈。
            return 0u;
        }
        if (elapsed < state->config.stable_ms)
            return 0u; // 未达到配置时间时保持禁止。
        state->command_baseline = input->command_sequence; // 稳定前的命令不能恢复后重放。
        state->phase = CAN_RECOVERY_WAIT_COMMAND; // 只接受之后到达的新命令。
        return 0u;
    }
    if (state->phase == CAN_RECOVERY_WAIT_COMMAND)
    {
        if (!SequenceIsNewer(input->command_sequence, state->command_baseline))
            return 0u; // 等待真正的新命令，不重复使用原值。
        state->command_baseline = input->command_sequence; // 记住已接受的命令，过期后不能重用。
        state->phase = CAN_RECOVERY_READY; // 反馈及新命令均已通过。
    }
    if (state->phase == CAN_RECOVERY_READY)
    {
        if (input->command_sequence == 0u ||
            (input->command_sequence != state->command_baseline &&
             !SequenceIsNewer(input->command_sequence, state->command_baseline)))
        {
            state->phase = CAN_RECOVERY_WAIT_COMMAND; // 命令过期或倒退立即禁止，直到收到更新的命令。
            return 0u;
        }
        state->command_baseline = input->command_sequence; // 新命令成为下一次恢复的比较基准。
    }
    return (uint8_t)(state->phase == CAN_RECOVERY_READY); // 未知阶段也不能授予许可。
}
