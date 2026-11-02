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

/** @brief 报告已启用温控模块的运行阶段初始化失败。
 * @return 1 表示影响核心模块，调用方必须关断整车；0 表示辅助模块降级。
 * @note 仅温控功能编译启用时提供；只在初始化失败时调用，不解除已有故障。
 */
uint8_t RobotReportHeaterInitFailure(void);
#endif
