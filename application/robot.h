/** @file robot.h
* @brief 整车初始化和周期入口，按编译配置选择单板、双板或裸板诊断流程。
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>

/* Robot利用robot_def.h中的宏对不同的机器人进行了大量的兼容,同时兼容了两个开发板(云台板和底盘板)的配置 */

/** @brief 在启动 RTOS 前初始化基础 BSP 并准备任务监控。
* @return 1 为准备成功，0 为失败；失败记录首次原因并关闭已知输出。
* @note main 调用一次；设备初始化和任务创建由调度器运行后的启动协调者完成。
 */
uint8_t RobotInit(void);

/** @brief 在调度器运行后按车型初始化已启用的业务模块。
* @return 核心模块成功返回 1；失败返回 0 并请求关断。
* @note 仅由启动协调者调用；初始化期间中断保持可用。
 */
uint8_t RobotInitializeApplications(void);

/** @brief 执行一轮所选应用，由 ROBOT 任务约每 5 ms 调用。
* @note 初始化失败时只更新诊断及失能状态；裸板配置只运行状态和视觉回传路径。
 */
void RobotTask();

#endif
