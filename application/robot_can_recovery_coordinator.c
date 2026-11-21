/** @file robot_can_recovery_coordinator.c
 * @brief 以固定容量组合单板人工控制的 CAN 恢复条件。
 */
#include "robot_can_recovery_coordinator.h"
#include "robot_can_runtime.h"
#include "robot_safety.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "platform_can.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include <stddef.h>

static RobotCanRecoveryCoordinator_t command_recovery; // 唯一命令任务的固定恢复存储。

uint8_t RobotCanRecoveryCoordinatorInit(RobotCanRecoveryCoordinator_t *state,
    const Robot_Config_s *config, const RobotModulePlan_t *modules)
{
    if (state == NULL || config == NULL || modules == NULL) return 0u; // 输入不完整时保留原实例。
    RobotCanRecoveryCoordinator_t candidate = {0}; // 默认没有恢复资格。
    if (config->can_recovery.confirmed == 0u)
    {
        *state = candidate; // 未确认参数保持禁用，不能用候选值授权实车。
        return 1u;
    }
    if (config->can_recovery.confirmed != 1u || !RobotCanMotorPlanBuild(config, modules, &candidate.motors))
        return 0u; // 非法确认值或无法识别核心电机时拒绝初始化。
    const CanRecoveryConfig_t timing = {
        .required_feedback_mask = (UINT32_C(1) << candidate.motors.count) - 1u, // 每个核心电机都有独立反馈位。
        .stable_ms = config->can_recovery.feedback_stable_ms, // 使用车型确认的毫秒值。
    };
    if (!CanRecoveryTimedInit(&candidate.timing, &timing)) return 0u; // 时间或反馈位无效时不交付。
    candidate.require_gimbal_feedback = (modules->enabled & ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL)) != 0u; // 纯底盘车型只检查已有核心来源。
    candidate.enabled = 1u; // 完整配置通过后才允许运行检查。
    *state = candidate; // 一次交付完整上下文。
    return 1u;
}

// 将本轮人工来源及其原始时间送入恢复检查，来源切换代次独立保留。
static void SelectManualSource(CanRecoveryTimedInput_t *input, const RemoteControlSnapshot_s *remote,
                               uint8_t keyboard_allowed)
{
    input->command_source_epoch = remote->source_selection_generation; // 短暂切换也要求重新等待。
    if (!remote->remote_online || remote->radio_sequence == 0u ||
        (uint32_t)(input->core.now_ms - remote->radio_source_time_ms) > REMOTE_RADIO_LEASE_MS) return; // 无线来源过期不得提供任何人工命令。
    if (switch_is_down(remote->input.rc.switch_left))
    {
        input->command_source_id = 1u; // 遥控摇杆使用无线帧的独立序号。
        input->core.command_sequence = remote->radio_sequence; // 重复读取不会生成新命令。
        input->command_source_ms = remote->radio_source_time_ms; // 保留原始接收时刻。
        input->command_max_age_ms = REMOTE_RADIO_LEASE_MS; // 不因重试而续期。
    }
    else if (keyboard_allowed && switch_is_up(remote->input.rc.switch_left) && remote->pc_valid)
    {
        input->command_source_id = 2u; // 键鼠来源与遥控摇杆分别比较。
        input->core.command_sequence = remote->pc_sample_sequence; // 键鼠心跳必须向前推进。
        input->command_source_ms = remote->pc_source_time_ms; // 保留键鼠自己的原始时刻。
        input->command_max_age_ms = REMOTE_PC_LEASE_MS; // 使用键鼠独立期限。
    }
}

// 同时检查命令任务复制的原期限，以及消息中心是否仍保留这一段有效反馈。
static uint8_t GimbalFeedbackIsCurrent(const MessageFreshness_t *feedback, uint32_t now_ms)
{
    return (uint8_t)(MessageFreshnessIsValid(feedback, now_ms, MESSAGE_DEFAULT_MAX_AGE_MS) &&
        MessageSnapshotSourceIsCurrent(ROBOT_TOPIC_GIMBAL_FEEDBACK, feedback->sequence, now_ms));
}

uint8_t RobotCanRecoveryCoordinatorStep(RobotCanRecoveryCoordinator_t *state,
    const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed, const MessageFreshness_t *application_feedback)
{
    if (state == NULL || remote == NULL || state->enabled != 1u) return 0u; // 禁用或无输入时保持故障。
    RobotSafetySnapshot_t safety = {0}; // 整轮使用同一次安全状态和代次。
    if (!RobotSafetyGetSnapshot(&safety)) return 0u; // 无法读取安全状态时不能恢复。
    if (!(safety.fault_flags & ROBOT_FAULT_CAN_BUSOFF))
    {
        state->generation_seen = 0u; // 下次故障必须重新捕获禁止前的反馈。
        state->feedback_gap_seen = 0u; // 下次故障重新读取设备间断代次。
        state->gimbal_feedback_sequence = 0u; // 不把本次故障的应用反馈带到下一次恢复。
        return 0u;
    }
    const uint8_t formal_autonomous = RobotSafetyIsFormalAutonomous(); // 本入口尚无自主命令来源。
    RobotCanReadiness_t bus = {0}; // 查询期间发生故障会使就绪位清零。
    const uint8_t bus_read = RobotCanRuntimeGetReadiness(&bus); // 在允许中断的任务上下文查询控制器。
    if (!state->generation_seen || state->can_generation != bus.fault_generation ||
        state->safety_generation != safety.control_generation)
    {
        ++state->inhibit_generation; // 任一禁止事件都重新开始反馈等待。
        state->can_generation = bus.fault_generation; // 记录本轮总线故障代次。
        state->safety_generation = safety.control_generation; // 记录本轮安全禁止代次。
        state->generation_seen = 1u; // 零也是合法代次。
    }
    CanRecoveryTimedInput_t input = {0}; // 任何读取失败默认无效。
    input.core.now_ms = PlatformTimeNowMs(); // 反馈和来源均使用本机毫秒。
    uint8_t application_ok = 1u; // 纯底盘车型不等待未启用的云台反馈。
    if (state->require_gimbal_feedback)
    {
        application_ok = GimbalFeedbackIsCurrent(application_feedback, input.core.now_ms); // 复制后撤销的反馈也会被拒绝。
        if (application_ok && state->gimbal_feedback_sequence != 0u &&
            state->gimbal_feedback_sequence != application_feedback->sequence &&
            !MessageSnapshotSourceIsCurrent(ROBOT_TOPIC_GIMBAL_FEEDBACK, state->gimbal_feedback_sequence, input.core.now_ms))
            ++state->inhibit_generation; // 两轮之间短暂中断时重新等待各路反馈。
        state->gimbal_feedback_sequence = application_ok ? application_feedback->sequence : 0u; // 记住本轮用于比较的区间。
    }
    input.core.inhibit_generation = state->inhibit_generation; // 短暂禁止不能被跳过。
    input.core.bus_ok = (uint8_t)(bus_read && bus.core_bus_mask != 0u && bus.core_bus_mask == bus.ready_bus_mask); // 每条核心总线都必须可用。
    input.core.permit_ok = (uint8_t)(application_ok && !formal_autonomous &&
        RobotSafetyCanRecover(safety.control_generation)); // 自主来源未接入时也中断已有的人工恢复等待。
    uint32_t registered_mask = 0u; // 最终检查仅覆盖车型的必需设备。
    if (DJIMotorGetRecoveryFeedback(input.core.now_ms, &state->feedback))
        (void)RobotCanMotorRecoveryMapWithMask(&state->motors, &state->feedback,
            input.core.now_ms, &input.core, &registered_mask); // 缺失或身份不匹配时反馈位保持无效。
    if (registered_mask != 0u)
    {
        for (uint8_t index = 0u; index < state->feedback.count; ++index)
        {
            if ((registered_mask & (UINT32_C(1) << index)) == 0u) continue; // 只核对车型要求的电机。
            const uint32_t gap = state->feedback.motors[index].gap_generation; // 按登记下标读取间断代次。
            if (state->feedback_gap_seen && state->feedback_gap[index] != gap)
                ++state->inhibit_generation; // 两次轮询之间的断流也必须重新等待。
            state->feedback_gap[index] = gap; // 保存当前代次供下轮比较。
        }
        state->feedback_gap_seen = 1u; // 后续即使回到健康状态也能识别曾发生的间断。
        input.core.inhibit_generation = state->inhibit_generation; // 交给状态机撤销旧的稳定时间。
    }
    SelectManualSource(&input, remote, keyboard_allowed); // 只选当前合法人工来源。
    if (!CanRecoveryTimedStep(&state->timing, &input)) return 0u; // 先等新反馈、稳定时间和新命令。
    const PlatformIrqState_t irq = PlatformIrqSaveAndDisable(); // 防止最终检查之间插入可屏蔽中断。
    uint8_t buses_ready = 1u; // 逐条复核此前识别的核心总线。
    for (unsigned index = 0u; index < 3u; ++index)
        if ((bus.core_bus_mask & (1u << index)) &&
            !PlatformCanRecoveryStillReady((PlatformCanBus_t)((unsigned)PLATFORM_CAN_BUS_1 + index)))
            buses_ready = 0u; // 任何控制器退出、隔离或报错都不能恢复。
    const uint32_t now = PlatformTimeNowMs(); // 控制器检查结束后再次读取期限所用时间。
    const uint8_t application_current = (uint8_t)(!state->require_gimbal_feedback ||
        GimbalFeedbackIsCurrent(application_feedback, now)); // 原期限和发布区间都必须保持有效。
    const uint8_t motors_current = DJIMotorRecoveryStillValid(&state->feedback, registered_mask, now); // 复核当前反馈、间断和设备身份。
    const uint8_t remote_current = RemoteControlRecoveryStillValid(remote, now); // 复核本次人工来源及其期限。
    uint8_t cleared = 0u; // 检查失败默认保持故障。
    if (buses_ready && application_current && motors_current && remote_current)
        cleared = RobotCanRuntimeTryClearFault(bus.fault_generation, safety.control_generation, now); // 最终原子复核故障代次和整车资格。
    PlatformIrqRestore(irq); // 恢复调用者原来的中断状态。
    if (!buses_ready || !application_current || !motors_current || !remote_current)
    {
        if (!application_current) state->gimbal_feedback_sequence = 0u; // 状态机重新等待，新区间从下一轮开始记录。
        input.core.bus_ok = 0u; // 硬件或应用反馈失效立即中断连续有效计时。
        (void)CanRecoveryTimedStep(&state->timing, &input); // 保存当前反馈作为新的比较起点。
    }
    return cleared; // 清除故障不直接发布电机输出。
}

uint8_t RobotCanRecoveryStart(const Robot_Config_s *config, const RobotModulePlan_t *modules)
{
    return RobotCanRecoveryCoordinatorInit(&command_recovery, config, modules); // 启动时完成配置检查。
}

uint8_t RobotCanRecoveryPoll(const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed,
                             const MessageFreshness_t *application_feedback)
{
    return RobotCanRecoveryCoordinatorStep(&command_recovery, remote, keyboard_allowed, application_feedback); // 由命令任务单独推进。
}
