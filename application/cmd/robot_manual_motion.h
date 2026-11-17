/** @file robot_manual_motion.h
 * @brief 将已校验的人工 SI 意图换算为当前底盘与云台应用的控制量。
 */
#ifndef MC02_ROBOT_MANUAL_MOTION_H
#define MC02_ROBOT_MANUAL_MOTION_H
#include "std_cmd.h"

/** @brief 当前周期控制量；底盘消息暂沿用旧单位与横向约定。 */
typedef struct {
    float vx_mm_s; /**< 底盘中心前进速度，mm/s。 */
    float vy_right_mm_s; /**< 底盘中心向右速度，mm/s。 */
    float wz_deg_s; /**< 底盘绕上轴角速度，deg/s。 */
    float offset_deg; /**< 输入相对底盘的方向角，deg；底盘参照为零。 */
    float yaw_deg; /**< 枪口整体 yaw 的本周期目标，deg。 */
    float pitch_deg; /**< 枪口 pitch 的本周期目标，deg。 */
} RobotManualMotion_t;

/** @brief 换算本次人工意图，不发布命令也不授予运行许可。
 * @param[in] intent 已由输入适配器检查的 SI 意图，不可为 NULL。
 * @param[in] now_ms 当前本机毫秒，用于再次检查来源期限。
 * @param[in] dt_ms 距离上次人工计算的实测间隔，单位 ms；0 表示本周期不积分。
 * @param[in] max_dt_ms 车型允许的最大间隔，单位 ms；0 或超出半个计数范围无效。
 * @param[in] big_yaw_offset_deg 大 yaw 相对于底盘的方向角，deg；大 yaw 参照时必须有限。
 * @param[in] yaw_reference_deg 上次已交付或按实际姿态建立的 yaw 参考，deg。
 * @param[in] pitch_reference_deg 上次已交付或按实际姿态建立的 pitch 参考，deg。
 * @param[out] output 成功时完整写入结果；失败时保持旧值。
 * @return 1 为所有检查及换算通过；来源过期、间隔超限、输入非有限或参照尚不支持返回 0。
 * @note 当前旧底盘应用只支持底盘和大 yaw 参照；枪口和场地参照应等待对应反馈与转换接线。
 * 启动、模式切换、反馈过期和禁止恢复后必须由调用者从当前姿态重建参考。
 */
uint8_t RobotManualMotionProject(const StdCmdManualIntent_t *intent, uint32_t now_ms,
    uint32_t dt_ms, uint32_t max_dt_ms, float big_yaw_offset_deg,
    float yaw_reference_deg, float pitch_reference_deg, RobotManualMotion_t *output);
#endif
