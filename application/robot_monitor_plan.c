/** @file robot_monitor_plan.c
 * @brief 用车型依赖决定姿态任务是否监控及其故障影响。
 */
#include "robot_monitor_plan.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include <stddef.h>

#define ROBOT_DIAGNOSTIC_PERIOD_MS 5u // 保留整车任务的名义周期，也用于监控的毫秒单位。

#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
static const PlatformPeriodicRequest_t diagnostic_periods[ROBOT_PERIODIC_COUNT] = {
    [ROBOT_PERIODIC_DIAGNOSTIC] = {
        .timing = {
            .period_us = ROBOT_DIAGNOSTIC_PERIOD_MS * 1000u, // 现有诊断周期，从 ms 转为 μs。
            .budget_us = ROBOT_DIAGNOSTIC_PERIOD_MS * 1000u, // 保留现有单轮耗时告警所用的预算。
        },
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 使用 FreeRTOS 精确表示这一间隔。
    },
#if MC02_BENCH_TIM5_DIAGNOSTICS
    [ROBOT_PERIODIC_TIM5_PROBE] = {
        .timing = {
            .period_us = 500u, // 裸板自测拍点，每 500 μs 一拍。
            .budget_us = 500u, // 自测只读取计数，单轮预算为 500 μs。
        },
        .source = PLATFORM_PERIODIC_SOURCE_HIGH_RES, // 显式验证 TIM5 通道及中断，即使 RTOS 刻度改变。
    },
#endif
};

PlatformPeriodicResult_t RobotValidatePeriodicDiagnostics(RobotPeriodicDiagnostic_t *failed)
{
    const PlatformPeriodicCapabilities_t capabilities = PlatformTaskPeriodicCapabilities(); // 读取实际唤醒能力。
    uint32_t hardware_mask; // 计划函数需完成整个分配；诊断项的后端已由配置指定。
    size_t failed_index = 0u; // 固定表首项用于报告通用配置错误。
    const PlatformPeriodicResult_t result = PlatformPeriodicPlan(diagnostic_periods, ROBOT_PERIODIC_COUNT,
        &capabilities, &hardware_mask, &failed_index); // 校验两种来源是否都能满足各自配置。
    if (result != PLATFORM_PERIODIC_READY && failed != NULL)
        *failed = (RobotPeriodicDiagnostic_t)failed_index;
    return result;
}

const PlatformPeriodicConfig_t *RobotGetDiagnosticPeriod(RobotPeriodicDiagnostic_t diagnostic)
{
    if ((unsigned)diagnostic >= ROBOT_PERIODIC_COUNT) return NULL; // 越界时不读取表外数据。
    return &diagnostic_periods[diagnostic].timing;
}
#endif

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
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = ROBOT_DIAGNOSTIC_PERIOD_MS,
     .deadline_ms = 20u, .startup_grace_ms = 100u}, // 整车任务，监控沿用以毫秒表达的期限。
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
