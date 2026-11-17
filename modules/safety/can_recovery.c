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

static uint8_t SequenceIsNewer(uint32_t current, uint32_t previous)
{
    if (current == 0u) return 0u; // 零保留给尚无样本或命令。
    if (previous == 0u) return 1u; // 之前尚无数据时，首个非零序号有效。
    const uint32_t distance = current - previous; // 按无符号差值处理序号回绕。
    return (uint8_t)(distance != 0u && distance <= INT32_MAX); // 相同、倒退或无法判断的序号均不能接管。
}

static void CaptureBaseline(CanRecovery_t *state, const CanRecoveryInput_t *input)
{
    for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
    {
        if (SequenceIsNewer(state->last_feedback[index], state->baseline[index]))
            state->baseline[index] = state->last_feedback[index]; // 缺样时仍保留上一次收到的序号。
        if (SequenceIsNewer(input->feedback_sequence[index], state->baseline[index]))
            state->baseline[index] = input->feedback_sequence[index]; // 只前移基准，缺样或旧值不能抹掉已经观察到的序号。
        state->last_feedback[index] = state->baseline[index]; // 后续比较从故障时最新序号开始。
    }
    state->seen_mask = 0u; // 每次禁止后都要求重新收到反馈。
    if (SequenceIsNewer(input->command_sequence, state->command_baseline))
        state->command_baseline = input->command_sequence; // 记录最近命令；无效输入不能把旧基准清零。
    state->inhibit_generation = input->inhibit_generation; // 记录当前已见的禁止事件代际。
    state->baseline_captured = 1u; // 后续可与新的反馈比较。
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
    for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
    {
        const uint32_t bit = UINT32_C(1) << index; // 只检查车型要求的反馈。
        if ((required & bit) == 0u) continue; // 未使用的序号不参与恢复。
        const uint32_t sequence = input->feedback_sequence[index]; // 读取本轮这一路的序号。
        if (sequence != state->last_feedback[index] && !SequenceIsNewer(sequence, state->last_feedback[index]))
        {
            CaptureBaseline(state, input); // 序号倒退可能是旧帧；不得撤销已观察到的较新基准。
            state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 重新等待全部必需反馈。
            return 0u;
        }
        state->last_feedback[index] = sequence; // 相同样本可继续读取，更新样本则推进记录。
    }
    if (state->phase != CAN_RECOVERY_WAIT_COMMAND && state->phase != CAN_RECOVERY_READY &&
        SequenceIsNewer(input->command_sequence, state->command_baseline))
        state->command_baseline = input->command_sequence; // 恢复期间出现的新命令同样不能在恢复后重放。
    if (state->phase == CAN_RECOVERY_WAIT_BUS)
        state->phase = CAN_RECOVERY_WAIT_FEEDBACK; // 总线恢复不直接授权运动。
    if (state->phase == CAN_RECOVERY_WAIT_FEEDBACK)
    {
        for (uint32_t index = 0u; index < CAN_RECOVERY_MAX_FEEDBACK; ++index)
        {
            const uint32_t bit = UINT32_C(1) << index; // 每一位对应同序号的必需反馈。
            if ((required & bit) != 0u)
            {
                if (SequenceIsNewer(input->feedback_sequence[index], state->baseline[index]))
                    state->seen_mask |= bit; // 当前样本必须比禁止时的新。
                else
                    state->seen_mask &= ~bit; // 回退到旧样本时撤销该路已更新标记。
            }
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
        if (SequenceIsNewer(input->command_sequence, state->command_baseline))
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

uint8_t CanRecoveryTimedInit(CanRecoveryTimed_t *state, const CanRecoveryConfig_t *config)
{
    if (state == NULL || config == NULL) return 0u; // 无效对象不能改写旧状态。
    CanRecoveryTimed_t candidate = {0}; // 新实例不继承旧来源和等待起点。
    if (!CanRecoveryInit(&candidate.recovery, config)) return 0u; // 先检查核心恢复配置。
    *state = candidate; // 全部配置成功后才交付实例。
    return 1u;
}

uint8_t CanRecoveryTimedStep(CanRecoveryTimed_t *state, const CanRecoveryTimedInput_t *input)
{
    if (state == NULL || input == NULL || state->recovery.initialized != 1u) return 0u; // 无效状态不能允许恢复。
    CanRecoveryInput_t current = input->core; // 保留调用方的逐台反馈和禁止代次。
    const uint8_t source_valid = (uint8_t)(input->command_source_id != 0u &&
        current.command_sequence != 0u && input->command_max_age_ms > 0u &&
        input->command_max_age_ms <= INT32_MAX &&
        (uint32_t)(current.now_ms - input->command_source_ms) <= input->command_max_age_ms); // 过期或将来的来源不能提供新命令。
    if (!source_valid) current.command_sequence = 0u; // 无有效来源时不能复用旧命令。
    if (source_valid && (input->command_source_id != state->source_id ||
                         input->command_source_epoch != state->source_epoch))
    {
        const CanRecoveryConfig_t config = state->recovery.config; // 来源切换时保留原反馈配置。
        if (!CanRecoveryInit(&state->recovery, &config)) return 0u; // 任何初始化错误都保持禁止。
        state->source_id = input->command_source_id; // 后续序号只和同一来源比较。
        state->source_epoch = input->command_source_epoch; // 记录来源选择代次，短暂切换也会重新等待。
        state->barrier_valid = 0u; // 新来源不能使用旧来源的稳定起点。
        (void)CanRecoveryStep(&state->recovery, &current); // 先记录切换时已存在的反馈和命令。
        return 0u; // 等待每台电机重新更新并保持稳定。
    }
    const CanRecoveryPhase_t before = state->recovery.phase; // 记录本轮推进前的阶段。
    if (state->barrier_valid && (before == CAN_RECOVERY_WAIT_COMMAND || before == CAN_RECOVERY_READY))
    {
        const uint32_t advance = input->command_source_ms - state->command_barrier_ms; // 仅接收稳定之后产生的原始命令。
        if (advance == 0u || advance > INT32_MAX) current.command_sequence = 0u; // 同时刻、旧时间或歧义时间不能接管。
    }
    const uint8_t ready = CanRecoveryStep(&state->recovery, &current); // 总线、逐台反馈及命令序号继续由原状态机检查。
    if (before == CAN_RECOVERY_STABILIZING && state->recovery.phase == CAN_RECOVERY_WAIT_COMMAND)
    {
        state->command_barrier_ms = current.now_ms; // 稳定完成当轮作为新命令的时间起点。
        state->barrier_valid = 1u; // 0 ms 起点同样有效。
    }
    if (state->recovery.phase == CAN_RECOVERY_WAIT_BUS || state->recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK)
        state->barrier_valid = 0u; // 故障或反馈失效后不能沿用上次起点。
    return (uint8_t)(ready && state->barrier_valid); // 两套检查均通过才交付恢复条件。
}
