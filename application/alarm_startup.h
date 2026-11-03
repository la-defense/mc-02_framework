/** @file alarm_startup.h
 * @brief 根据模块依赖决定告警通道降级时能否继续启动。
 */
#ifndef MC02_ALARM_STARTUP_H
#define MC02_ALARM_STARTUP_H
#include <stdint.h>
#include "robot_alarm.h"
#include "module_plan.h"

/** @brief 初始化告警并按依赖范围处理部分通道失效。
 * @param[in] config 长期只读告警配置，可为空。
 * @param[in] plan 已验证的模块计划，不能为 NULL。
 * @return 1 为允许告警任务运行；0 为配置错误或核心能力不完整。
 * @note 辅助告警降级会记录初始化事件，核心失败由启动计划统一记录。
 */
uint8_t RobotStartAlarmConfigured(const RobotAlarmConfig_t *config, const RobotModulePlan_t *plan);
#endif
