/** @file robot_can_requirements.h
 * @brief 将当前车型的设备配置转换为 CAN 过滤器需求。
 */
#ifndef MC02_ROBOT_CAN_REQUIREMENTS_H
#define MC02_ROBOT_CAN_REQUIREMENTS_H
#include "config.h"
#include "robot_can_capacity.h"

/** @brief 汇总本板启用设备的端点；调用前须完成车型配置校验。
 * @param[in] config 非空的只读车型配置。
 * @param[out] demands 至少容纳 8 项的数组。
 * @return 需求项数，最多 7 项；不注册设备。
 */
uint8_t RobotCollectCanDemands(const Robot_Config_s *config, RobotCanFilterDemand_t demands[8]);
#endif
