/** @file std_cmd.c
 * @brief 人工输入的单位换算、期限检查和按次事件去重；不依赖HAL或RTOS。
 */
#include "std_cmd.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

// 期限必须小于半个计数范围，使无符号差值可以区分将来时刻与正常回绕。
static uint8_t ValidAge(uint32_t age)
{
    return (uint8_t)(age > 0u && age <= INT32_MAX);
}

StdCmdStatus_t StdCmdAdaptManual(const StdCmdManualMap_t *map,
                                const StdCmdManualInput_t *input,
                                uint32_t now_ms, StdCmdManualIntent_t *output)
{
    // 在解引用前检查全部对象，失败时不接触调用者输出。
    if (map == NULL || input == NULL || output == NULL)
        return STD_CMD_BAD_ARGUMENT;
    // 允许位只覆盖已有参照；未配置期限或全部禁用时拒绝映射。
    if (!ValidAge(map->max_age_ms) || map->allowed_frames == 0u ||
        (map->allowed_frames & ~((1u << STD_CMD_FRAME_COUNT) - 1u)) != 0u)
        return STD_CMD_BAD_CONFIG;
    // 先按通道检查范围和系数，防止无效配置被过期输入掩盖。
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
    {
        // 每次仅检查当前通道的三个物理换算参数。
        const StdCmdAxisMap_t *value = &map->axis[axis];
        // 原始范围必须正，输出可配置为零；无穷和NaN均不能进入乘法。
        if (!isfinite(value->input_limit) || value->input_limit <= 0.0f ||
            !isfinite(value->gain) || !isfinite(value->output_limit) || value->output_limit < 0.0f)
            return STD_CMD_BAD_CONFIG;
    }
    // 先验证枚举，再使用frame构造位移，避免非法枚举造成未定义行为。
    if (!input->valid || input->sequence == 0u || !ValidAge(input->max_age_ms) ||
        (unsigned)input->source >= STD_CMD_MANUAL_SOURCE_COUNT ||
        (unsigned)input->frame >= STD_CMD_FRAME_COUNT)
        return STD_CMD_INVALID_INPUT;
    // 车型没有允许的参照不能取得该来源运动意图。
    if ((map->allowed_frames & (1u << input->frame)) == 0u)
        return STD_CMD_INVALID_INPUT;
    // 两处期限取较小值，配置不能替来源数据延长使用时间。
    const uint32_t age = map->max_age_ms < input->max_age_ms ? map->max_age_ms : input->max_age_ms;
    // 将来时间的无符号差值也超期；正常计数回绕仍按实际小差值处理。
    if ((uint32_t)(now_ms - input->source_time_ms) > age)
        return STD_CMD_EXPIRED;
    // 先在局部结果中计算，任一后续通道失败时不交付部分新数据。
    StdCmdManualIntent_t candidate = {0};
    // 逐通道执行原始范围检查、SI换算及输出幅值限制。
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
    {
        // 取与本通道对应的车型映射。
        const StdCmdAxisMap_t *value = &map->axis[axis];
        // 越界原始数据直接拒绝，不能用限幅掩盖来源协议或标定错误。
        if (!isfinite(input->axis[axis]) || fabsf(input->axis[axis]) > value->input_limit)
            return STD_CMD_INVALID_INPUT;
        // 按有符号比例换算，安装和输入方向由车型指定。
        float converted = input->axis[axis] * value->gain;
        // 有限输入相乘仍可能溢出，溢出时不把结果当作可限幅的正常命令。
        if (!isfinite(converted))
            return STD_CMD_INVALID_INPUT;
        // 正向目标超过本通道允许幅值时取上限。
        if (converted > value->output_limit) converted = value->output_limit;
        // 负向目标按同样幅值限制，保持输入方向。
        if (converted < -value->output_limit) converted = -value->output_limit;
        // 保存当前已验证的SI目标，尚不发布给调用者。
        candidate.axis[axis] = converted;
    }
    candidate.source_time_ms = input->source_time_ms; // 保留原始时间，重复换算不续期。
    candidate.sequence = input->sequence; // 保留原始序号，重复读取不产生新命令。
    candidate.max_age_ms = age; // 输出只能使用两处期限中的较小值。
    candidate.source = input->source; // 后续仲裁仍能识别摇杆与键鼠来源。
    candidate.frame = input->frame; // 下游根据同一方向参照转换运动量。
    *output = candidate; // 全部通道成功后一次交付。
    return STD_CMD_OK;
}

StdCmdStatus_t StdCmdConsumeEvent(StdCmdEventCursor_t *cursor,
                                 const StdCmdEvent_t *event, uint32_t now_ms)
{
    // 对象不存在时不能查询或推进消费序号。
    if (cursor == NULL || event == NULL)
        return STD_CMD_BAD_ARGUMENT;
    // 验证来源后才能用作数组下标；零序号及非法期限不表示有效事件。
    if ((unsigned)event->source >= STD_CMD_MANUAL_SOURCE_COUNT ||
        event->sequence == 0u || !ValidAge(event->max_age_ms))
        return STD_CMD_INVALID_INPUT;
    // 过期事件不消费序号，调用方可观察拒绝原因但不能执行动作。
    if ((uint32_t)(now_ms - event->source_time_ms) > event->max_age_ms)
        return STD_CMD_EXPIRED;
    // 分别保存各来源的已消费号，切换来源不会将另一来源误当重复。
    const uint32_t previous = cursor->sequence[event->source];
    // 无符号前进量允许32位回绕；重复、后退和半区间歧义均拒绝。
    const uint32_t advance = event->sequence - previous;
    // 初次非零序号可以接受；已有序号时必须在半个计数范围内向前。
    if (previous != 0u && (advance == 0u || advance > INT32_MAX))
        return STD_CMD_DUPLICATE;
    // 只在成功接受后推进序号，由单一消费者保证同事件只执行一次。
    cursor->sequence[event->source] = event->sequence;
    return STD_CMD_OK;
}
