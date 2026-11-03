/** @file alarm_startup.c
 * @brief 保留可用告警通道，并区分核心依赖和辅助降级。
 */
#include "alarm_startup.h"
#include "init_diagnostic.h"
#include <stddef.h>

uint8_t RobotStartAlarmConfigured(const RobotAlarmConfig_t *config, const RobotModulePlan_t *plan)
{
    if (config == NULL || plan == NULL)
        return 0u; // 缺少配置或依赖计划时不能启动。
    const RobotAlarmStatus_t status = RobotAlarmInit(config); // 初始化车型选择的灯声通道。
    if (status == ROBOT_ALARM_DEGRADED)
    {
        const uint32_t alarm_bit = ROBOT_MODULE_BIT(ROBOT_MODULE_ALARM); // 查找告警在依赖图中的位置。
        if ((RobotModuleCoreClosure(plan) & alarm_bit) != 0u)
            return 0u; // 核心功能要求完整能力，交由启动计划锁存失败。
        InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                     INIT_FAILURE_RESOURCE, ROBOT_MODULE_ALARM); // 记录辅助通道不可用。
        return 1u; // 保留可用通道并允许任务继续处理。
    }
    return (uint8_t)(status == ROBOT_ALARM_OK); // 配置错误仍视为启动失败。
}
