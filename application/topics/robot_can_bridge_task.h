/** @file robot_can_bridge_task.h
 * @brief 双板话题桥接在通信任务中的轻量服务入口。
 */
#ifndef MC02_ROBOT_CAN_BRIDGE_TASK_H
#define MC02_ROBOT_CAN_BRIDGE_TASK_H
/** @brief 串行推进话题收发及对时检查。
 * @note 每次最多提交一个当前话题；对时无效时接收侧保持失效。
 */
void RobotCanBridgeService(void);
/** @brief 对本板桥接总线额外推进至多一帧。 */
void RobotCanBridgeServiceTx(void);
#endif
