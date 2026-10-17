/** @file robot.h
* @brief 整车初始化和周期入口，按编译配置选择单板、双板或裸板诊断流程。
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>

/* Robot利用robot_def.h中的宏对不同的机器人进行了大量的兼容,同时兼容了两个开发板(云台板和底盘板)的配置 */

/** @brief 在启动 RTOS 前初始化 BSP、应用和任务，保留并恢复调用前的普通中断状态。
* @return 1 为初始化和任务创建成功，0 为失败；失败记录首次原因并关闭已知输出，不启动后续控制。
* @note main 调用一次；部分设备和 INS 算法由其专属任务在调度器运行后继续初始化。
 */
uint8_t RobotInit(void);

/** @brief 执行一轮所选应用，由 ROBOT 任务约每 5 ms 调用。
* @note 初始化失败时只更新诊断及失能状态；裸板配置只运行状态和视觉回传路径。
 */
void RobotTask();

#endif
