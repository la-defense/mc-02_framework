/** @file commissioning_chassis_config.c
 * @brief 在设备启动前检查受限底盘所需的人工映射、时间和变化率。
 */
#include "config.h"
#include "std_cmd_remote.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

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
    return (uint8_t)(isfinite(linear_rate * max_dt_s) && isfinite(angular_rate * max_dt_s)); // 两项上限必须在允许的最长周期内仍可计算。
}
