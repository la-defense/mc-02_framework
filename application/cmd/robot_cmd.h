/** @file robot_cmd.h
 * @brief 汇总遥控、视觉和反馈，生成底盘、云台、发射消息，再由整车状态检查是否允许执行。
 */
#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#include <stdint.h>


/** @brief 注册遥控、视觉和控制话题，初始控制量保持为零。
 * @return 1 为本配置所需注册全部成功，0 为失败；由 RobotInit() 在 RTOS 启动前调用一次。
 */
uint8_t RobotCMDInit(void);

/** @brief 运行一轮命令选择与发布，初始化成功后由 RobotTask() 约每 5 ms 调用。
 * @note 云台反馈过期或数值不可用时发布停止命令；视觉退出后先从当前姿态接续手动目标。
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H
