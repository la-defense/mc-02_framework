/** @file log_startup.h
 * @brief 按车型依赖检查已初始化的日志出口。
 */
#ifndef MC02_LOG_STARTUP_H
#define MC02_LOG_STARTUP_H
#include "module_plan.h"
#include <stdint.h>

/** @brief 检查 RTT 日志出口是否满足车型模块计划。
 * @param[in] plan 已验证的模块计划；空指针返回失败。
 * @return 1 表示日志已就绪，或仅作为辅助功能时允许降级；0 表示核心依赖未满足。
 * @note 独立诊断串口的失败在基础启动中单独记录，不决定 RTT 日志的可用性。
 */
uint8_t RobotStartLogConfigured(const RobotModulePlan_t *plan);
#endif
