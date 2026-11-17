/** @file robot_manual_motion.c
 * @brief 显式转换人工输入的单位、横向方向与周期角度增量。
 */
#include "robot_manual_motion.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>

uint8_t RobotManualMotionProject(const StdCmdManualIntent_t *intent, uint32_t now_ms,
    uint32_t dt_ms, uint32_t max_dt_ms, float big_yaw_offset_deg,
    float yaw_reference_deg, float pitch_reference_deg, RobotManualMotion_t *output)
{
    if (intent == NULL || output == NULL || intent->sequence == 0u || intent->max_age_ms == 0u ||
        intent->max_age_ms > INT32_MAX || max_dt_ms == 0u || max_dt_ms > INT32_MAX || dt_ms > max_dt_ms)
        return 0u; // 对象、来源或实测间隔不合法时不交付运动目标。
    if ((uint32_t)(now_ms - intent->source_time_ms) > intent->max_age_ms)
        return 0u; // 来源过期或处于无法解释的将来时刻时撤销目标。
    if (intent->frame != STD_CMD_FRAME_CHASSIS && intent->frame != STD_CMD_FRAME_BIG_YAW)
        return 0u; // 旧应用尚无枪口和场地参照转换，不能忽略参照含义。
    if (!isfinite(yaw_reference_deg) || !isfinite(pitch_reference_deg) ||
        (intent->frame == STD_CMD_FRAME_BIG_YAW && !isfinite(big_yaw_offset_deg)))
        return 0u; // 姿态参考或所需方向角无效时不得积分。
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
        if (!isfinite(intent->axis[axis])) return 0u; // 非有限的任意一个通道都会使整份意图无效。
    const float degrees_per_radian = 57.29577951308232f; // 弧度换算为当前应用使用的度。
    const float dt_s = (float)dt_ms * 0.001f; // 按实测毫秒间隔计算秒数。
    const RobotManualMotion_t candidate = {
        .vx_mm_s = intent->axis[STD_CMD_VX] * 1000.0f, // 米每秒换成旧底盘消息的毫米每秒。
        .vy_right_mm_s = -intent->axis[STD_CMD_VY] * 1000.0f, // SI 向左为正，旧消息向右为正。
        .wz_deg_s = intent->axis[STD_CMD_WZ] * degrees_per_radian, // 弧度每秒换成度每秒。
        .offset_deg = intent->frame == STD_CMD_FRAME_BIG_YAW ? big_yaw_offset_deg : 0.0f, // 底盘参照不能再旋转一次。
        .yaw_deg = yaw_reference_deg + intent->axis[STD_CMD_YAW_RATE] * dt_s * degrees_per_radian, // 用实际间隔积分整体 yaw。
        .pitch_deg = pitch_reference_deg + intent->axis[STD_CMD_PITCH_RATE] * dt_s * degrees_per_radian, // pitch 使用同一时间间隔。
    };
    if (!isfinite(candidate.vx_mm_s) || !isfinite(candidate.vy_right_mm_s) ||
        !isfinite(candidate.wz_deg_s) || !isfinite(candidate.yaw_deg) || !isfinite(candidate.pitch_deg))
        return 0u; // 有限输入相乘或相加仍可能溢出，不能发布部分结果。
    *output = candidate; // 所有计算通过后一次交付。
    return 1u;
}
