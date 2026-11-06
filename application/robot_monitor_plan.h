/** @file robot_monitor_plan.h
 * @brief 根据车型模块依赖准备当前任务监控配置。
 */
#ifndef MC02_ROBOT_MONITOR_PLAN_H
#define MC02_ROBOT_MONITOR_PLAN_H
#include "config/module_plan.h"
#include "task_monitor.h"

/** @brief 将车型依赖映射到已选择任务的核心／辅助监控。
 * @param[in] plan 车型模块计划；生成配置必须非空，旧配置可传 NULL。
 * @return 任务监控初始化结果；生成配置无效时不初始化监控。
 * @note 只在调度器启动前调用；监控实现会复制临时配置。
 */
TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan);
#endif
