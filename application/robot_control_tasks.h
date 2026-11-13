/** @file robot_control_tasks.h
 * @brief 整车协调使用的底盘、云台和发射业务入口。
 */
#ifndef MC02_ROBOT_CONTROL_TASKS_H
#define MC02_ROBOT_CONTROL_TASKS_H
#include <stdint.h>

/** @brief 初始化底盘资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 由启动协调任务在调度器运行后调用一次。
 */
uint8_t ChassisInit(void);
/** @brief 查询最近一次底盘初始化是否发生资源回退失败。
 * @return 非零表示有资源无法安全撤销，启动协调者必须整体失能。
 * @note 在启动协调任务中串行读取；每次 ChassisInit 开始时清除上一结果。
 */
uint8_t ChassisInitRollbackFailed(void);
/** @brief 在成功初始化后执行一轮底盘控制；超过 50 ms 的命令失效并停止相应输出。 */
void ChassisTask(void);
/** @brief 初始化云台资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 由启动协调任务调用一次；INS 随后由协调任务初始化。
 */
uint8_t GimbalInit(void);
/** @brief 查询最近一次云台初始化是否发生资源回退失败。
 * @return 非零表示有资源无法安全撤销，启动协调者必须整体失能。
 * @note 在启动协调任务中串行读取；每次 GimbalInit 开始时清除上一结果。
 */
uint8_t GimbalInitRollbackFailed(void);
/** @brief 在成功初始化后执行一轮云台控制；超过 50 ms 的命令失效并停止相应输出。 */
void GimbalTask(void);
/** @brief 初始化发射机构资源。
 * @return 成功返回 1，资源或话题注册失败返回 0。
 * @note 由启动协调任务调用一次，须先准备车型发射配置。
 */
uint8_t ShootInit(void);
/** @brief 查询最近一次发射初始化是否发生资源回退失败。
 * @return 非零表示仍有资源无法安全撤销，启动协调者必须整体失能。
 * @note 在启动协调任务中串行读取；每次 ShootInit 开始时清除上一结果。
 */
uint8_t ShootInitRollbackFailed(void);
/** @brief 在成功初始化后执行一轮发射控制；超过 50 ms 的命令失效并停止相应输出。 */
void ShootTask(void);
#endif
