/**
 * @file robot_topics_diag.h
 * @brief 声明通过 USART10 按请求读取消息中心话题统计的启动入口。
 * @note 仅在robot配置中使用；先完成BSPLogInit和RobotTopicsInit，再在调度器启动前注册。
 */
#ifndef MC02_ROBOT_TOPICS_DIAG_H
#define MC02_ROBOT_TOPICS_DIAG_H

#include "bsp_log.h"

/** @brief 注册话题统计诊断命令；不会建立周期发送任务。
 * @return BSP_LOG_STATUS_OK或日志模块返回的注册错误。
 */
BSPLogStatus_t RobotTopicsDiagnosticInit(void);

#endif
