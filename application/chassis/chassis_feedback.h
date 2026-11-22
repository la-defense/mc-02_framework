/** @file chassis_feedback.h
 * @brief 只读底盘速度与里程计反馈。
 */
#ifndef MC02_CHASSIS_FEEDBACK_H
#define MC02_CHASSIS_FEEDBACK_H

#include "chassis_kinematics.h"


/** @brief 由四轮反馈估算的底盘速度，使用前、左、上坐标。 */
typedef struct {
    ChassisTwist_t twist; /**< 底盘中心平移速度 m/s，绕竖轴转速 rad/s。 */
    uint32_t source_time_ms; /**< 参与计算的最早轮速接收时间，0 是合法时间。 */
} ChassisVelocityFeedback_t;

/** @brief 读取本板最近一次有效的麦轮或四轮全向轮底盘速度估计。
 * @param[out] output 底盘中心速度及最早来源时间；失败时非空输出清零。
 * @return 1 表示仍在原有效期内且四轮反馈连续，0 表示缺少估计或任一来源失效。
 * @note 任务上下文使用；四轮估计以滚动模型为依据，无法单独识别打滑。
 */
uint8_t Chassis_GetVelocityFeedback(ChassisVelocityFeedback_t *output);

/** @brief 本地轮速里程计，只在同一段内使用相同的参考原点。 */
typedef struct {
    ChassisLocalPose_t pose; /**< 从该段起点累计的位移和朝向。 */
    uint32_t source_time_ms; /**< 最近一次参与积分的最早轮速接收时间。 */
    uint32_t segment; /**< 断流或超出允许间隔后重新起算，每段加 1，允许回绕。 */
} ChassisOdometryFeedback_t;

/** @brief 复制可选的本地轮速里程计，供任务上下文使用。
 * @param[out] output 位姿、来源时间及段号；失败时非空输出清零。
 * @return 1 表示已启用且轮速来源有效；0 表示关闭、未取得样本或已失效。
 * @note 首批有效反馈的位姿为零。之后四轮都收到新样本且最早时刻前进才积分。
 *       任一轮断流后会立即停止读出旧位姿，恢复后从新段起算。本地位姿会受轮组打滑影响。
 */
uint8_t Chassis_GetOdometry(ChassisOdometryFeedback_t *output);

/** @brief 可同时读取速度和位姿，使用同一次读出时刻和四轮连续性检查。
 * @param[out] velocity 速度及来源时间，传 NULL 可跳过；缺测时清零。
 * @param[out] odometry 位姿、来源时间及段号，传 NULL 可跳过；缺测时清零。
 * @return 有效项的位图，速度为 bit 0，位姿为 bit 1；两者均无效或未请求时返回 0。
 * @note 普通任务调用。两项的来源时刻可以不同。
 */
uint8_t Chassis_GetMotionFeedback(ChassisVelocityFeedback_t *velocity, ChassisOdometryFeedback_t *odometry);

#endif
