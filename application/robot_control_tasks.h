/** @file robot_control_tasks.h
 * @brief 整车协调使用的底盘、云台和发射业务入口。
 */
#ifndef MC02_ROBOT_CONTROL_TASKS_H
#define MC02_ROBOT_CONTROL_TASKS_H
#include <stdint.h>

/** @brief 初始化底盘资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 在调度器启动前调用一次。
 */
uint8_t ChassisInit(void);
/** @brief 在成功初始化后执行一轮底盘控制；超过 50 ms 的命令失效并停止相应输出。 */
void ChassisTask(void);
/** @brief 初始化云台资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 在调度器启动前调用一次；INS 随后由专属任务启动。
 */
uint8_t GimbalInit(void);
/** @brief 在成功初始化后执行一轮云台控制；超过 50 ms 的命令失效并停止相应输出。 */
void GimbalTask(void);
/** @brief 初始化发射机构资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 在调度器启动前调用一次，须先准备车型发射配置。
 */
uint8_t ShootInit(void);
/** @brief 在成功初始化后执行一轮发射控制；超过 50 ms 的命令失效并停止相应输出。 */
void ShootTask(void);
#endif
