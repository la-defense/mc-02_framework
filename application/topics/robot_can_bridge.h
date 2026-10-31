/** @file robot_can_bridge.h
 * @brief 双板底盘话题的CAN链路启动与任务服务入口。
 * @note 板间对时尚未有效或数据过期时，远端话题不可用于控制。
 */
#ifndef ROBOT_CAN_BRIDGE_H
#define ROBOT_CAN_BRIDGE_H
#include <stdint.h>
#include "time_mapping.h"
/** @brief 注册本板的CAN和话题端点。
 * @return 1为成功，0为资源或配置错误；调度器启动前串行调用一次。
 */
uint8_t RobotCanBridgeInit(void);
/** @brief 在CAN接收派发任务中推进发送与同步检查。
 * @note 每次最多提交一个当前话题；对时未有效时接收侧保持失效。
 */
void RobotCanBridgeService(void);
/** @brief 复制最近一次板间对时状态，供其他普通任务查询。
 * @param[out] output 偏差、样本数及同步标记；失败保持原值。
 * @param[out] updated_ms 此副本的本机更新时间；失败保持原值。
 * @return 1为已复制，0为空参数或任务尚未发布。
 * @note 超过5ms未更新或样本超过500ms时，同步标记返回0；不可在中断中调用。
 */
uint8_t RobotCanBridgeGetTimeSyncState(TimeMappingState_t *output, uint32_t *updated_ms);
/** @brief 对本板专用桥接总线额外推进至多一帧；由电机任务串行调用。 */
void RobotCanBridgeServiceTx(void);
#endif
