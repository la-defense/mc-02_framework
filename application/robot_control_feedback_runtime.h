/** @file robot_control_feedback_runtime.h
 * @brief 单板整车反馈的启动绑定和周期提交入口。
 */
#ifndef MC02_ROBOT_CONTROL_FEEDBACK_RUNTIME_H
#define MC02_ROBOT_CONTROL_FEEDBACK_RUNTIME_H
#include "robot_control_feedback.h"

/** @brief 在启动协调任务中固定电机清单并绑定安全管理的反馈来源。
 * @param[in] plan 本板全部启用机构的必需电机，函数复制清单。
 * @return 1 为成功；0 为参数、清单或启动状态错误，启动者须保持失能。
 */
uint8_t RobotControlFeedbackStart(const RobotCanMotorPlan_t *plan);

/** @brief 由命令任务汇总反馈并提交，在发布前再次检查各组电机。
 * @param[in] pose 本轮有效的云台消息时间，NULL 表示姿态或编码器无效。
 * @return 1 为安全管理接受了本轮状态；0 为状态未配置或无法发布。
 * @note 单一命令任务调用。没有设置任何模式、使能请求或运动目标。
 */
uint8_t RobotControlFeedbackPoll(const MessageFreshness_t *pose);
#endif
