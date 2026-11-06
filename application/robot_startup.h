/** @file robot_startup.h
 * @brief 调度器运行后的整车启动协调入口。
 */
#ifndef MC02_ROBOT_STARTUP_H
#define MC02_ROBOT_STARTUP_H
#include <stdint.h>

/** @brief 完成设备初始化、任务创建及周期任务放行。
 * @return 全部核心步骤成功返回 1，失败返回 0 并锁存失能。
 * @note 由唯一默认任务调用一次；调用前已配置任务监控并延迟放行。
 */
uint8_t RobotStartupRun(void);
#endif
