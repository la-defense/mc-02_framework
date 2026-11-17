/** @file std_cmd_remote.c
 * @brief 按车型选择摇杆或键盘通道，保留来源期限后调用人工意图换算。
 */
#include "std_cmd_remote.h"
#include <stddef.h>

// 已验证索引后按枚举读取原始摇杆，不在这里改变方向或限幅。
static float ReadRemoteAxis(const RC_ctrl_t *input, StdCmdRemoteAxis_t axis)
{
    switch (axis)
    {
    case STD_CMD_RC_LEFT_X: return (float)input->rc.rocker_l_;
    case STD_CMD_RC_LEFT_Y: return (float)input->rc.rocker_l1;
    case STD_CMD_RC_RIGHT_X: return (float)input->rc.rocker_r_;
    case STD_CMD_RC_RIGHT_Y: return (float)input->rc.rocker_r1;
    case STD_CMD_RC_DIAL: return (float)input->rc.dial;
    default: return 0.0f; // 零通道不产生运动；未知枚举在调用前已拒绝。
    }
}

uint8_t StdCmdRemoteMapIsValid(const StdCmdRemoteMap_t *map)
{
    if (map == NULL) return 0u; // 没有配置不能启动人工控制。
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
        if ((unsigned)map->remote_axis[axis] >= STD_CMD_RC_AXIS_COUNT ||
            (map->keyboard_positive[axis] & map->keyboard_negative[axis]) != 0u)
            return 0u; // 通道越界或同一键对应正反方向均无效。
    StdCmdManualInput_t probe = {.valid = 1u, .sequence = 1u, .source_time_ms = 1u,
        .max_age_ms = REMOTE_RADIO_LEASE_MS, .source = STD_CMD_REMOTE, .frame = map->remote_frame}; // 只用零输入检查映射，不执行动作。
    StdCmdManualIntent_t result = {0}; // 验证结果只保存在局部。
    if (StdCmdAdaptManual(&map->remote_map, &probe, 1u, &result) != STD_CMD_OK) return 0u; // 摇杆映射和方向参照都必须合法。
    probe.source = STD_CMD_KEY_MOUSE; // 键鼠使用自己的映射和期限。
    probe.frame = map->keyboard_frame;
    probe.max_age_ms = REMOTE_PC_LEASE_MS;
    return (uint8_t)(StdCmdAdaptManual(&map->keyboard_map, &probe, 1u, &result) == STD_CMD_OK); // 两种来源均可检查后才允许启动。
}

StdCmdStatus_t StdCmdAdaptRemote(const StdCmdRemoteMap_t *map, const RemoteControlSnapshot_s *remote,
                                 uint32_t now_ms, StdCmdManualIntent_t *output)
{
    if (map == NULL || remote == NULL || output == NULL) return STD_CMD_BAD_ARGUMENT; // 输入不全不能改写旧输出。
    if (!StdCmdRemoteMapIsValid(map)) return STD_CMD_BAD_CONFIG; // 两种来源的通道及参数都必须完整。
    if (!remote->remote_online || remote->radio_sequence == 0u ||
        (uint32_t)(now_ms - remote->radio_source_time_ms) > REMOTE_RADIO_LEASE_MS)
        return STD_CMD_INVALID_INPUT; // 无线离线或来源过期时，键盘和摇杆都不可使用。
    StdCmdManualInput_t input = {0}; // 按当前档位构造完整的原始输入。
    const StdCmdManualMap_t *selected_map = NULL; // 每个来源使用自己的转换和期限配置。
    input.valid = 1u; // 遥控层已解析协议；本模块还要检查当前档位。
    if (switch_is_down(remote->input.rc.switch_left))
    {
        selected_map = &map->remote_map; // 摇杆的原始单位由该映射换算。
        input.source = STD_CMD_REMOTE; // 后续可以识别命令来源。
        input.frame = map->remote_frame; // 保留车型声明的参照。
        input.source_time_ms = remote->radio_source_time_ms; // 沿用最初接收时刻。
        input.sequence = remote->radio_sequence; // 同一帧重复读取不会变成新命令。
        input.max_age_ms = REMOTE_RADIO_LEASE_MS; // 不能延长无线来源期限。
        for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
            input.axis[axis] = ReadRemoteAxis(&remote->input, map->remote_axis[axis]); // 各轴按车型选择输入。
    }
    else if (switch_is_up(remote->input.rc.switch_left) && remote->pc_valid)
    {
        selected_map = &map->keyboard_map; // 持续按键使用独立比例和期限。
        input.source = STD_CMD_KEY_MOUSE; // 键盘与无线帧不能混用时间。
        input.frame = map->keyboard_frame; // 使用键盘声明的参照。
        input.source_time_ms = remote->pc_source_time_ms; // 使用键鼠心跳的原始时刻。
        input.sequence = remote->pc_sample_sequence; // 只随键鼠来源更新推进。
        const uint32_t pc_age = now_ms - remote->pc_source_time_ms; // 按原始时刻计算键盘已使用的时间。
        if (pc_age > REMOTE_PC_LEASE_MS) return STD_CMD_EXPIRED; // 键盘过期或时刻在未来时不能转换。
        const uint32_t radio_remaining = REMOTE_RADIO_LEASE_MS - (now_ms - remote->radio_source_time_ms); // 无线剩余期限不能由键盘延长。
        const uint32_t combined_age = pc_age + radio_remaining; // 换算成以键盘来源时刻为起点的最大年龄。
        input.max_age_ms = combined_age < REMOTE_PC_LEASE_MS ? combined_age : REMOTE_PC_LEASE_MS; // 使用更早到期的来源。
        if (input.max_age_ms == 0u) return STD_CMD_EXPIRED; // 无法用正期限表达的边界保持禁止。
        const uint16_t keys = remote->input.key[KEY_PRESS].keys; // 只读取本次快照中的按键状态。
        for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
        {
            const int positive = (keys & map->keyboard_positive[axis]) != 0u; // 任一正向键按住时请求正向。
            const int negative = (keys & map->keyboard_negative[axis]) != 0u; // 任一负向键按住时请求负向。
            input.axis[axis] = (float)(positive - negative); // 正反同时按下时归零。
        }
    }
    else return STD_CMD_INVALID_INPUT; // 中档或未授权键鼠不得产生人工目标。
    return StdCmdAdaptManual(selected_map, &input, now_ms, output); // 转换仍检查参数、数值与来源期限。
}
