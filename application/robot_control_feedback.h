/** @file robot_control_feedback.h
 * @brief 根据车型必需电机及云台反馈生成分功能的有效期限。
 */
#ifndef MC02_ROBOT_CONTROL_FEEDBACK_H
#define MC02_ROBOT_CONTROL_FEEDBACK_H
#include "robot_can_motor_recovery.h"
#include "robot_control_policy.h"
#include "message_freshness.h"

/** @brief 单一协调任务持有的设备清单与反馈间断记录。 */
typedef struct {
    RobotCanMotorPlan_t motors; /**< 按车型身份保存的全部所需电机。 */
    uint32_t gap_generation[DJI_MOTOR_CNT]; /**< 每个清单位置上次观察到的间断代次。 */
    uint32_t sample_sequence[DJI_MOTOR_CNT]; /**< 每台设备最近接受的非零样本序号。 */
    uint32_t source_time_ms[DJI_MOTOR_CNT]; /**< 每台设备最近接受的原始毫秒时刻。 */
    RobotControlLease_t last_lease[3]; /**< 各功能上次交付的期限；同一电机样本不能延长。 */
    uint32_t gap_seen; /**< 已观察到样本的清单位置。 */
    uint8_t initialized; /**< 完整检查通过后才允许汇总。 */
} RobotControlFeedback_t;

/** @brief 本轮各功能的反馈状态；数组顺序为底盘、云台、发射。 */
typedef struct {
    uint32_t valid_functions; /**< 每个有效位对应 ROBOT_CONTROL_*；不存在的功能不会被标为有效。 */
    RobotControlLease_t feedback[3]; /**< 保留各功能最早到期条件，不因汇总续期。 */
    uint32_t registered_motor_mask[3]; /**< 各功能在实际登记表中的下标，供最终复核。 */
} RobotControlFeedbackResult_t;

/** @brief 固定车型设备身份和反馈要求，不访问设备。
 * @param[out] state 单一任务持有的实例；失败不改写。
 * @param[in] plan 完整的已选设备清单，不能为空且至少有一台。
 * @return 1 为成功，0 为容量、身份或用途无效。
 */
uint8_t RobotControlFeedbackInit(RobotControlFeedback_t *state, const RobotCanMotorPlan_t *plan);

/** @brief 按功能逐台匹配并检查反馈；云台还需要当前姿态消息期限。
 * @param[in,out] state 已初始化的单一任务状态；记录两次读取之间的反馈间断。
 * @param[in] actual 本轮已登记 DJI 电机及原始反馈，不能为空。
 * @param[in] gimbal_pose 云台应用反馈的本机时间；NULL 表示云台反馈无效。
 * @param[in] now_ms 与电机和姿态使用同一时钟域的本机毫秒，允许回绕。
 * @param[out] output 完整结果；参数错误时保持原值。
 * @return 1 为汇总成功，0 为对象或容量错误；无有效功能也是成功的全零结果。
 * @note 不发布许可；最终提交前还须复核登记电机、姿态数据和安全状态。
 * 非 DJI 设备、机械限位及其他功能依赖须由调用者另行检查。
 */
uint8_t RobotControlFeedbackCollect(RobotControlFeedback_t *state, const DJI_RecoveryFeedback_s *actual,
    const MessageFreshness_t *gimbal_pose, uint32_t now_ms, RobotControlFeedbackResult_t *output);
#endif
