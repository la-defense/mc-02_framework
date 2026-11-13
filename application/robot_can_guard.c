/** @file robot_can_guard.c
 * @brief 根据现有车型启动计划处理诊断后的 CAN 隔离。
 */
#include "robot_can_guard.h"
#include "config/vehicle_selection.h"
#include "platform_can.h"
#include "init_diagnostic.h"
#include <stddef.h>

uint8_t RobotCanGuardCheck(const RobotCanDependency_t *dependencies, uint8_t count)
{
    if ((count != 0u && dependencies == NULL) || count > 8u) return 0u; // 拒绝无效或超出固定容量的列表。
    for (uint8_t i = 0u; i < count; ++i)
    {
        const RobotCanDependency_t item = dependencies[i]; // 读取当前模块与总线。
        if ((unsigned)item.module >= ROBOT_MODULE_COUNT) return 0u; // 非法模块不能被静默忽略。
        if (!RobotBuildModuleStarted(item.module)) continue; // 已失败或未启用的模块不重复处理。
        if (!PlatformCanBusIsQuarantined(item.bus)) continue; // 没有诊断隔离时不改变启动结果。
        if (RobotBuildMarkFailure(item.module)) // 传播失败并检查是否影响核心闭包。
        {
            InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                        INIT_FAILURE_RESOURCE, (uint16_t)item.bus); // 保留受影响的总线。
            return 0u; // 核心依赖失败时禁止创建和放行任务。
        }
        InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                     INIT_FAILURE_RESOURCE, (uint16_t)item.bus); // 仅辅助模块受影响。
    }
    return 1u;
}
