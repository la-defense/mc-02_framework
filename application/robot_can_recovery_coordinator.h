/** @file robot_can_recovery_coordinator.h
 * @brief 将车型要求、人工输入及实际电机反馈组合为 CAN 恢复检查。
 */
#ifndef MC02_ROBOT_CAN_RECOVERY_COORDINATOR_H
#define MC02_ROBOT_CAN_RECOVERY_COORDINATOR_H
#include "robot_can_motor_recovery.h"
#include "remote_control.h"
#include "message_freshness.h"

/** @brief 由命令任务独占的恢复上下文；不要在多个任务中并发调用。 */
typedef struct {
    RobotCanMotorPlan_t motors; /**< 启动时确定的核心电机身份。 */
    CanRecoveryTimed_t timing; /**< 逐台反馈和新命令的等待状态。 */
    DJI_RecoveryFeedback_s feedback; /**< 上次读取的实际登记设备，供最终复核。 */
    uint32_t can_generation; /**< 上一轮观察到的核心 CAN 故障代次。 */
    uint32_t safety_generation; /**< 上一轮观察到的整车禁止代次。 */
    uint32_t inhibit_generation; /**< 任一禁止代次变化时推进。 */
    uint32_t feedback_gap[DJI_MOTOR_CNT]; /**< 各实际登记设备上次观察到的反馈间断代次。 */
    uint8_t feedback_gap_seen; /**< 区分首次读取与代次恰好为零。 */
    uint8_t generation_seen; /**< 区分尚未读取和代次恰好为零。 */
    uint8_t require_gimbal_feedback; /**< 车型启用云台时还需验证云台应用反馈。 */
    uint8_t enabled; /**< 只有车型确认恢复时间且设备计划合法时才启用。 */
} RobotCanRecoveryCoordinator_t;

/** @brief 初始化单板人工来源的恢复检查，不操作总线或解除故障。
 * @param[out] state 单一命令任务持有的上下文；失败保持旧值。
 * @param[in] config 非空车型配置，未确认时初始化为禁用。
 * @param[in] modules 非空模块依赖计划。
 * @return 1 为成功或按配置禁用，0 为非法配置或无法构造必需设备清单。
 */
uint8_t RobotCanRecoveryCoordinatorInit(RobotCanRecoveryCoordinator_t *state,
    const Robot_Config_s *config, const RobotModulePlan_t *modules);

/** @brief 推进一次人工模式恢复，满足全部条件后尝试清除 CAN 故障。
 * @param[in,out] state 启动时初始化、由当前任务独占的上下文。
 * @param[in] remote 本轮已读取的遥控与键鼠状态，不能为空。
 * @param[in] keyboard_allowed 1 表示当前协议允许键鼠档为人工运动来源。
 * @param[in] application_feedback 云台反馈原始期限；只有不启用云台的车型允许没有该数据。
 * @return 1 为本轮成功清除 CAN 故障；其他情况返回 0 并继续禁止。
 * @note 应用反馈失效时仍须调用以中断连续等待；正式自主来源尚未接入本入口。
 */
uint8_t RobotCanRecoveryCoordinatorStep(RobotCanRecoveryCoordinator_t *state,
    const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed, const MessageFreshness_t *application_feedback);

/** @brief 启动时为单板命令任务准备唯一恢复上下文。
 * @param[in] config 只读车型配置。
 * @param[in] modules 已检查的模块计划。
 * @return 1 为初始化成功或配置为禁用，0 为确认的配置无效。
 */
uint8_t RobotCanRecoveryStart(const Robot_Config_s *config, const RobotModulePlan_t *modules);

/** @brief 由单一命令任务推进启动时创建的恢复上下文。
 * @param[in] remote 当前人工输入。
 * @param[in] keyboard_allowed 1 表示键鼠档是人工来源。
 * @param[in] application_feedback 云台反馈原始期限；只有不启用云台的车型允许没有该数据。
 * @return 1 为本轮清除 CAN 故障，0 为继续等待。
 */
uint8_t RobotCanRecoveryPoll(const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed,
                             const MessageFreshness_t *application_feedback);
#endif
