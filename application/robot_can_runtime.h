/** @file robot_can_runtime.h
 * @brief 将车型核心模块依赖的 CAN 总线故障送入整车安全管理。
 */
#ifndef MC02_ROBOT_CAN_RUNTIME_H
#define MC02_ROBOT_CAN_RUNTIME_H
#include "robot_can_capacity.h"

/** @brief 根据启动计划安装核心总线故障观察者。
 * @param[in] plan 已声明的模块计划，不可为 NULL。
 * @param[in] demands 端点需求数组；count 为零时可为 NULL。
 * @param[in] count 需求数量，最多为 8。
 * @return 1 为配置成功或没有核心 CAN 需求；0 为输入无效、总线无绑定或观察者安装失败。
 * @note 启动阶段调用一次；已报告的故障不会因控制器重启而自动清除。
 */
uint8_t RobotCanRuntimeInstall(const RobotModulePlan_t *plan,
                               const RobotCanFilterDemand_t *demands, uint8_t count);
#endif
