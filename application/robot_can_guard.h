/** @file robot_can_guard.h
 * @brief 在诊断后检查已启动模块对隔离 CAN 总线的依赖。
 */
#ifndef MC02_ROBOT_CAN_GUARD_H
#define MC02_ROBOT_CAN_GUARD_H
#include "module_plan.h"
#include "platform_resources.h"
#include <stdint.h>

typedef struct {
    RobotModule_t module; /**< 已声明的使用模块。 */
    PlatformCanBus_t bus; /**< 该模块使用的逻辑 CAN 总线。 */
} RobotCanDependency_t;

/** @brief 将诊断隔离传播到已启动模块及其依赖者。
 * @param[in] dependencies 固定容量依赖数组；数量为零时可为 NULL。
 * @param[in] count 数组中的有效项数。
 * @return 1 表示核心依赖未受影响；无效输入或核心受影响返回 0。
 * @note 仅在启动协调者创建周期任务之前调用；不检查设备在线状态。
 */
uint8_t RobotCanGuardCheck(const RobotCanDependency_t *dependencies, uint8_t count);
#endif
