/** @file robot_control_feedback.c
 * @brief 逐台验证 DJI 电机及云台反馈，分功能保留最早到期时间。
 */
#include "robot_control_feedback.h"
#include "message_center.h"
#include <limits.h>
#include <stddef.h>

// 机构用途只映射到已定义的三个功能；未声明用途不能推断。
static unsigned FunctionIndex(MotorOutputRole_t role)
{
    if (role == MOTOR_OUTPUT_CHASSIS) return 0u;
    if (role == MOTOR_OUTPUT_GIMBAL) return 1u;
    if (role == MOTOR_OUTPUT_FRICTION || role == MOTOR_OUTPUT_LOADER) return 2u;
    return 3u;
}

// 与 DJI 驱动相同地确定标准反馈 ID，避免不同型号占用同一接收身份。
static uint32_t FeedbackId(Motor_Type_e type, uint32_t id)
{
    return type == GM6020 ? 0x204u + id : 0x200u + id;
}

uint8_t RobotControlFeedbackInit(RobotControlFeedback_t *state, const RobotCanMotorPlan_t *plan)
{
    if (state == NULL || plan == NULL || plan->count == 0u || plan->count > DJI_MOTOR_CNT)
        return 0u; // 空清单或超容量不能表达有效反馈。
    for (uint8_t index = 0u; index < plan->count; ++index)
    {
        const RobotCanMotorRequirement_t *motor = &plan->motors[index]; // 逐台验证身份和用途。
        if (FunctionIndex(motor->role) >= 3u || motor->bus < PLATFORM_CAN_BUS_1 ||
            motor->bus > PLATFORM_CAN_BUS_3 || motor->id == 0u || motor->id > 8u ||
            (motor->type != M2006 && motor->type != M3508 && motor->type != GM6020) ||
            motor->require_position > 1u) return 0u; // 未实现型号和无效身份不能取得许可。
        for (uint8_t previous = 0u; previous < index; ++previous)
        {
            const RobotCanMotorRequirement_t *other = &plan->motors[previous]; // 检查重复的预期身份。
            if (motor->bus == other->bus && FeedbackId(motor->type, motor->id) == FeedbackId(other->type, other->id))
                return 0u; // 同总线相同反馈 ID 不能被两个用途共同认领。
        }
    }
    RobotControlFeedback_t candidate = {0}; // 新实例不继承旧间断代次。
    candidate.motors = *plan; // 启动后清单保持固定。
    candidate.initialized = 1u; // 全部通过才允许读取。
    *state = candidate; // 成功后一次交付。
    return 1u;
}

// 用云台消息的两个独立期限缩短电机时间对应的允许年龄。
static uint8_t ConstrainPose(const MessageFreshness_t *pose, uint32_t now_ms,
                             uint32_t oldest_motor_ms, uint32_t *max_age_ms)
{
    if (pose == NULL || (pose->source_clock != MESSAGE_CLOCK_LOCAL && pose->source_clock != MESSAGE_CLOCK_LOCAL_MAPPED) ||
        pose->source_max_age_ms > INT32_MAX || !MessageFreshnessIsValid(pose, now_ms, MESSAGE_DEFAULT_MAX_AGE_MS))
        return 0u; // 未映射的跨设备时间、非法期限或过期消息不能授权云台。
    const uint32_t motor_age = now_ms - oldest_motor_ms; // 电机样本到当前时刻的实际年龄。
    const uint32_t topic_age = now_ms - (pose->source_max_age_ms ? pose->published_time_ms : pose->timestamp_ms); // 与消息中心相同的发布时间规则。
    uint32_t remaining = MESSAGE_DEFAULT_MAX_AGE_MS - topic_age; // 话题还可使用的时间。
    if (pose->source_max_age_ms != 0u)
    {
        const uint32_t source_remaining = pose->source_max_age_ms - (now_ms - pose->source_time_ms); // 来源自己的剩余时间。
        if (source_remaining < remaining) remaining = source_remaining; // 两个期限取更早到期者。
    }
    const uint32_t allowed_age = motor_age + remaining; // 换算为以最早电机采样为起点的年龄。
    if (allowed_age == 0u) return 0u; // 安全管理的期限需要正值，边界无法表达时保守拒绝。
    if (allowed_age < *max_age_ms) *max_age_ms = allowed_age; // 从不延长电机反馈自身期限。
    return 1u;
}

uint8_t RobotControlFeedbackCollect(RobotControlFeedback_t *state, const DJI_RecoveryFeedback_s *actual,
    const MessageFreshness_t *gimbal_pose, uint32_t now_ms, RobotControlFeedbackResult_t *output)
{
    if (state == NULL || state->initialized != 1u || state->motors.count == 0u ||
        state->motors.count > DJI_MOTOR_CNT || actual == NULL || output == NULL ||
        actual->count > DJI_MOTOR_CNT) return 0u; // 对象或容量错误不改写结果。
    RobotControlFeedbackResult_t result = {0}; // 未确认的功能一律保持无效。
    uint32_t present = 0u, invalid = 0u, used = 0u; // 分别记录车型需要、故障及已匹配设备。
    uint32_t oldest_age[3] = {0u, 0u, 0u}; // 每个功能最早电机样本的当前年龄。
    for (uint8_t index = 0u; index < state->motors.count; ++index)
    {
        const RobotCanMotorRequirement_t *expected = &state->motors.motors[index]; // 读取固定清单的一台电机。
        const unsigned function = FunctionIndex(expected->role); // 每台设备只属于一个功能。
        const uint32_t bit = UINT32_C(1) << function; // 功能位与安全管理保持一致。
        present |= bit; // 此功能至少要求一台电机。
        uint8_t matches = 0u, slot = 0u; // 实际登记身份必须恰好出现一次。
        for (uint8_t candidate = 0u; candidate < actual->count; ++candidate)
        {
            const DJI_RecoveryFeedbackItem_s *item = &actual->motors[candidate]; // 比较注册时的身份。
            if (item->bus == expected->bus && item->role == expected->role &&
                item->motor_type == expected->type && item->motor_id == expected->id)
            { ++matches; slot = candidate; } // 记录匹配个数，重复身份不能被接受。
        }
        if (matches != 1u || (used & (UINT32_C(1) << slot)) != 0u)
        { invalid |= bit; continue; } // 缺失、重复或重复占用只使对应功能失效。
        used |= UINT32_C(1) << slot; // 后续额外设备检查使用登记下标。
        result.registered_motor_mask[function] |= UINT32_C(1) << slot; // 保留最终复核的身份范围。
        const DJI_RecoveryFeedbackItem_s *item = &actual->motors[slot]; // 读取匹配设备的样本。
        const uint32_t sample_age = now_ms - item->source_time_ms; // 保留原始时间，不用汇总时刻续期。
        if (item->sequence == 0u || !item->speed_valid || !item->current_valid ||
            (expected->require_position && !item->position_valid) || sample_age > DJI_FEEDBACK_MAX_AGE_MS)
        { invalid |= bit; continue; } // 速度、电流或必需位置过期时撤销该功能。
        const uint32_t expected_bit = UINT32_C(1) << index; // 间断代次按固定清单位置保存。
        if (state->gap_seen & expected_bit)
        {
            const uint32_t advance = item->sequence - state->sample_sequence[index]; // 比较此电机自己的样本序号。
            const uint32_t time_advance = item->source_time_ms - state->source_time_ms[index]; // 原始时间也必须向前或相同。
            if (advance > INT32_MAX || time_advance > INT32_MAX ||
                (advance == 0u && time_advance != 0u))
            { invalid |= bit; continue; } // 旧帧或同一序号改写时刻不得覆盖已接受的数据。
        }
        if ((state->gap_seen & expected_bit) && state->gap_generation[index] != item->gap_generation)
            invalid |= bit; // 即使现在恢复，两轮之间发生过断流也必须报告失效边沿。
        state->gap_generation[index] = item->gap_generation; // 保存本次有效样本的间断代次。
        state->sample_sequence[index] = item->sequence; // 后续读取须保持相同或向前。
        state->source_time_ms[index] = item->source_time_ms; // 防止某一电机的旧时间被其他设备掩盖。
        state->gap_seen |= expected_bit; // 标记此位置已有可比较的样本。
        if (!result.feedback[function].valid || sample_age > oldest_age[function])
        {
            oldest_age[function] = sample_age; // 相同设备期限下，年龄最大的电机最早到期。
            result.feedback[function] = (RobotControlLease_t){item->source_time_ms, DJI_FEEDBACK_MAX_AGE_MS, 1u}; // 原始采样时间随功能一起交付。
        }
    }
    for (uint8_t slot = 0u; slot < actual->count; ++slot)
    {
        const unsigned function = FunctionIndex(actual->motors[slot].role); // 检查同机构是否存在未声明设备。
        if (function >= 3u) { invalid |= present; continue; } // 未声明用途无法证明不会影响任何已选功能。
        if (function < 3u && (present & (UINT32_C(1) << function)) && !(used & (UINT32_C(1) << slot)))
            invalid |= UINT32_C(1) << function; // 不能漏掉该机构的额外已登记电机。
    }
    if ((present & ROBOT_CONTROL_GIMBAL) && !(invalid & ROBOT_CONTROL_GIMBAL) &&
        !ConstrainPose(gimbal_pose, now_ms, result.feedback[1].source_time_ms, &result.feedback[1].max_age_ms))
        invalid |= ROBOT_CONTROL_GIMBAL; // 云台目标依赖姿态时也须有独立的新鲜数据。
    result.valid_functions = present & ~invalid; // 只发布全部必需设备通过的功能。
    for (unsigned function = 0u; function < 3u; ++function)
    {
        const uint32_t bit = UINT32_C(1) << function; // 对每组保持与安全管理相同的功能位。
        RobotControlLease_t *current = &result.feedback[function]; // 当前由电机和其他反馈共同限制的期限。
        const RobotControlLease_t *last = &state->last_lease[function]; // 上次交付的原始时间与期限。
        if ((result.valid_functions & bit) && last->valid && current->source_time_ms == last->source_time_ms &&
            current->max_age_ms > last->max_age_ms)
            current->max_age_ms = last->max_age_ms; // 姿态更新不能替同一电机样本延长期限。
        if ((result.valid_functions & bit) && (uint32_t)(now_ms - current->source_time_ms) > current->max_age_ms)
            result.valid_functions &= ~bit; // 原期限已过时等待新的电机样本。
        if ((result.valid_functions & (UINT32_C(1) << function)) == 0u)
        {
            result.feedback[function] = (RobotControlLease_t){0}; // 无效功能不能沿用旧期限。
            result.registered_motor_mask[function] = 0u; // 不把不完整清单交给最终复核。
        }
        else state->last_lease[function] = *current; // 只有有效结果才更新防续期基准。
    }
    *output = result; // 一次交付完整的分功能结果。
    return 1u;
}
