/** @file robot_monitor_plan.c
 * @brief 用车型依赖决定姿态任务是否监控及其故障影响。
 */
#include "robot_monitor_plan.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include <stddef.h>

#if defined(MC02_CONFIG_GENERATED)
// 辅助姿态任务运行失败时撤销该模块及其下游启动结果。
static void RobotAuxiliaryInsTaskFailed(TaskMonitor_Id_e id, uint32_t dependencies, void *context)
{
    (void)context; // 当前车型不需要额外回调上下文。
    if (id != TASK_MONITOR_INS || !(dependencies & ROBOT_MODULE_BIT(ROBOT_MODULE_IMU))) return; // 只处理本配置的姿态故障。
    (void)RobotBuildMarkFailure(ROBOT_MODULE_IMU); // 依赖图标记后，业务循环不会继续调用受影响模块。
}
#endif

// 周期单位为毫秒；首次任务启动额外保留 100 ms 宽限。
static const TaskMonitorTaskConfig_t robot_task_monitor_config[] = {
    {.id = TASK_MONITOR_INS, .name = "INS", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u, .startup_grace_ms = 100u}, // 姿态周期。
#if MC02_FEATURE_COMM_DISPATCH
    {.id = TASK_MONITOR_MOTOR, .name = "MOTOR", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u, .startup_grace_ms = 100u}, // 电机及通信派发。
#endif
    {.id = TASK_MONITOR_ROBOT, .name = "ROBOT", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 5u, .deadline_ms = 20u, .startup_grace_ms = 100u}, // 整车控制。
    {.id = TASK_MONITOR_DAEMON, .name = "DAEMON", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 10u, .deadline_ms = 30u, .startup_grace_ms = 100u}, // 设备及任务监控。
    {.id = TASK_MONITOR_UART_TX, .name = "UART_TX", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 30u, .startup_grace_ms = 100u}, // 串口发送。
};

TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan)
{
    TaskMonitorTaskConfig_t selected[sizeof(robot_task_monitor_config) /
                                      sizeof(robot_task_monitor_config[0])]; // 使用固定容量。
    uint32_t count = 0u; // 按原顺序保存实际需要的监控。
#if defined(MC02_CONFIG_GENERATED)
    if (RobotModulePlanValidate(plan, NULL) != ROBOT_MODULE_OK)
        return TASK_MONITOR_RESULT_BAD_CONFIGURATION; // 依赖无效时不能猜测核心角色。
    const uint32_t core_modules = RobotModuleCoreClosure(plan); // 包括核心模块的全部必要依赖。
#else
    (void)plan; // 旧配置沿用原有固定监控。
#endif
    for (uint32_t index = 0u; index < (uint32_t)(sizeof(robot_task_monitor_config) /
                                                   sizeof(robot_task_monitor_config[0])); ++index)
    {
        TaskMonitorTaskConfig_t item = robot_task_monitor_config[index]; // 复制一项固定任务定义。
#if defined(MC02_CONFIG_GENERATED)
        if (item.id == TASK_MONITOR_INS)
        {
            if (!(plan->enabled & ROBOT_MODULE_BIT(ROBOT_MODULE_IMU))) continue; // 未启用时不注册姿态监控。
            if (!(core_modules & ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)))
            {
                item.role = TASK_MONITOR_ROLE_AUXILIARY; // 未被核心依赖时只报告辅助降级。
                item.dependency_mask = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 保存受影响模块编号。
                item.degrade_callback = RobotAuxiliaryInsTaskFailed; // 故障发生后撤销姿态模块及下游。
            }
        }
#endif
        selected[count++] = item; // 只交付该车型实际需要的监控。
    }
    return TaskMonitorInitConfigured(selected, count); // 监控模块复制配置，局部数组可释放。
}
