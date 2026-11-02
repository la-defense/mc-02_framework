/** @file robot_task_create.h
 * @brief 启动阶段创建所选任务并报告创建结果。
 */
#ifndef MC02_ROBOT_TASK_CREATE_H
#define MC02_ROBOT_TASK_CREATE_H
#include <stdint.h>

/** @brief 创建当前配置需要的任务。
 * @return 1 表示核心任务创建成功；0 表示核心任务失败。
 * @note 调度器启动前调用一次；辅助失败按车型依赖报告，不删除已创建任务。
 */
uint8_t OSTaskInit(void);
#endif
