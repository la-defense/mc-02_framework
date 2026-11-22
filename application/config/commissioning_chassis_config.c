/** @file commissioning_chassis_config.c
 * @brief 在设备启动前检查受限底盘所需的人工映射、时间和变化率。
 */
#include "config.h"
#include "robot_manual_motion.h"
#include "std_cmd_remote.h"
#include "topics/robot_topics.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

// 检查启用的底盘轴能把最大合法输入换算成 SI 速度，再换成旧底盘消息的单位。
static uint8_t CommissioningMotionCanBeRepresented(const StdCmdRemoteMap_t *map,
    StdCmdManualSource_t source, uint32_t max_dt_ms)
{
    const uint8_t is_radio = source == STD_CMD_REMOTE; // 无线和键盘使用不同的原始值范围。
    const StdCmdManualMap_t *mapping = is_radio ? &map->remote_map : &map->keyboard_map; // 取本来源的换算和幅值限制。
    const float radio_limit = fmaxf((float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET),
        (float)(RC_CH_VALUE_OFFSET - RC_CH_VALUE_MIN)); // DBUS 解码后摇杆和拨轮能达到的最大绝对值。
    StdCmdManualInput_t probe = {
        .source_time_ms = 1u, // 用固定时刻离线检查，启动时无需收到遥控帧。
        .sequence = 1u, // 换算接口要求序号非零。
        .max_age_ms = 1u, // 只用同一时刻检查换算，不参与实际许可。
        .source = source, // 与当前检查的映射一致。
        .frame = STD_CMD_FRAME_CHASSIS, // 受限底盘只使用本体坐标。
        .valid = 1u, // 让换算检查通道和数值。
    };
    for (unsigned axis = STD_CMD_VX; axis <= STD_CMD_WZ; ++axis) // 只检查受限底盘处理的三个速度轴。
    {
        if (is_radio && map->remote_axis[axis] != STD_CMD_RC_ZERO)
            probe.axis[axis] = fminf(radio_limit, mapping->axis[axis].input_limit); // 只试算协议和车型同时允许的最大值。
        if (!is_radio && (map->keyboard_positive[axis] | map->keyboard_negative[axis]) != 0u)
            probe.axis[axis] = 1.0f; // 实际按键只能产生 -1、0、1，幅值检查用等效的正向值。
    }
    StdCmdManualIntent_t intent; // 保留通道实际限幅后的 m/s 和 rad/s。
    RobotManualMotion_t projected; // 检查旧底盘消息的单位换算，不向应用发布。
    return StdCmdAdaptManual(mapping, &probe, probe.source_time_ms, &intent) == STD_CMD_OK && // 检查限幅之前的乘法及原始范围。
        RobotManualMotionProject(&intent, probe.source_time_ms, 0u, max_dt_ms,
                                 0.0f, 0.0f, 0.0f, &projected); // 检查限幅之后的底盘消息单位，无姿态积分。
}

uint8_t RobotCommissioningChassisInputIsValid(const RobotManualInputConfig_s *input)
{
    if (input == NULL || input->max_dt_ms == 0u || input->max_dt_ms > INT32_MAX ||
        !StdCmdRemoteMapIsValid(input->map) || input->map->remote_frame != STD_CMD_FRAME_CHASSIS ||
        input->map->keyboard_frame != STD_CMD_FRAME_CHASSIS)
        return 0u; // 受限底盘只接受完整的人工映射和本体坐标，不会取得云台或场地角度。

    const float linear_rate = input->commissioning_linear_accel_m_s2; // 平移速度向量每秒的最大变化量。
    const float angular_rate = input->commissioning_angular_accel_rad_s2; // 绕上轴角速度每秒的最大变化量。
    if (!isfinite(linear_rate) || linear_rate <= 0.0f || !isfinite(angular_rate) || angular_rate <= 0.0f)
        return 0u; // 零值表示尚未确认，负值或非有限值无法作为变化率上限。

    const float max_dt_s = (float)input->max_dt_ms * 0.001f; // 用运行时相同的毫秒换算计算最大单次步长。
    if (!isfinite(linear_rate * max_dt_s) || !isfinite(angular_rate * max_dt_s))
        return 0u; // 两项上限必须在允许的最长周期内仍可计算。
    return CommissioningMotionCanBeRepresented(input->map, STD_CMD_REMOTE, input->max_dt_ms) &&
        CommissioningMotionCanBeRepresented(input->map, STD_CMD_KEY_MOUSE, input->max_dt_ms); // 两种人工来源都会在运行中使用。
}

uint8_t RobotCommissioningChassisInputFitsPeriod(const RobotManualInputConfig_s *input,
                                               uint32_t command_period_us)
{
    if (input == NULL || input->map == NULL || command_period_us == 0u)
        return 0u; // 尚未选择输入或任务周期时无法比较期限。

    const uint32_t period_ms = command_period_us / 1000u + (command_period_us % 1000u != 0u); // 毫秒期限须覆盖整个微秒周期。
    const StdCmdRemoteMap_t *map = input->map; // 两种人工来源可能使用不同的车型期限。
    return (uint8_t)(period_ms <= input->max_dt_ms && // 下一次发布前须保留计算速度变化的时间基准。
        period_ms <= map->remote_map.max_age_ms && // 无线来源必须同时满足车型期限和协议期限。
        period_ms <= map->keyboard_map.max_age_ms && // 键鼠来源也受自身映射期限限制。
        period_ms <= REMOTE_RADIO_LEASE_MS && // 两种模式都需要无线在线。
        period_ms <= REMOTE_PC_LEASE_MS); // 键鼠还需独立的 PC 来源在线。
}

uint8_t RobotCommissioningChassisTimingFits(const RobotManualInputConfig_s *input,
    const PlatformPeriodicConfig_t *command, const PlatformPeriodicConfig_t *chassis,
    const MotorTaskPeriods_t *motor, const PlatformPeriodicConfig_t *dispatch)
{
    if (input == NULL || input->map == NULL || command == NULL || chassis == NULL ||
        motor == NULL || dispatch == NULL || command->budget_us == 0u || chassis->budget_us == 0u ||
        dispatch->budget_us == 0u || command->budget_us > command->period_us ||
        chassis->budget_us > chassis->period_us || dispatch->budget_us > dispatch->period_us ||
        motor->compute_us == 0u || motor->transmit_us == 0u)
        return 0u; // 缺少任一拍点或预算时不能估算整条路径。

    const uint64_t after_publish_us = (uint64_t)chassis->period_us + chassis->budget_us + // 等待底盘任务并提交轮速参考。
        motor->compute_us + motor->transmit_us + dispatch->budget_us; // 等待两个电机阶段，再给最终发送轮次留执行预算。
    const uint64_t from_input_us = (uint64_t)command->period_us + command->budget_us + after_publish_us; // 从合法输入可读取时算起，逐段相加避免溢出。
    const StdCmdRemoteMap_t *map = input->map; // 人工摇杆和键鼠各有配置期限。
    return (uint8_t)(after_publish_us <= (uint64_t)ROBOT_CHASSIS_COMMAND_MAX_AGE_MS * 1000u && // 话题期限从发布起算。
        from_input_us <= (uint64_t)map->remote_map.max_age_ms * 1000u && // 摇杆使用车型期限。
        from_input_us <= (uint64_t)map->keyboard_map.max_age_ms * 1000u && // 键鼠也必须能走完这条路径。
        from_input_us <= (uint64_t)REMOTE_RADIO_LEASE_MS * 1000u && // 两种来源都受无线帧期限限制。
        from_input_us <= (uint64_t)REMOTE_PC_LEASE_MS * 1000u); // 键鼠同时依赖 PC 帧的原始期限。
}
