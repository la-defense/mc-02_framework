/** @file robot_monitor_plan.c
 * @brief 定义周期任务的时间配置并按车型依赖选择监控项。
 */
#include "robot_monitor_plan.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include <stddef.h>
#include <limits.h>

#define ROBOT_DIAGNOSTIC_PERIOD_MS 5u // 整车诊断的名义周期，单位 ms。
#define ROBOT_INS_PERIOD_MS 1u // 姿态任务的名义周期，单位 ms。
#define ROBOT_COMM_PERIOD_MS 1u // 通信派发的名义周期，单位 ms。
#define ROBOT_DAEMON_PERIOD_MS 10u // 设备维护与监控的名义周期，单位 ms。

#if defined(MC02_MONITOR_CONFIGURED_CAPACITY)
_Static_assert(MC02_MONITOR_CONFIGURED_CAPACITY == TASK_MONITOR_COUNT + MC02_TASK_CHASSIS + MC02_TASK_GIMBAL + MC02_TASK_SHOOTER,
               "firmware task monitor capacity must match its task selection");
#endif

static const PlatformPeriodicRequest_t task_periods[ROBOT_PERIODIC_COUNT] = {
    [ROBOT_PERIODIC_DIAGNOSTIC] = {
        .timing = {
            .period_us = ROBOT_DIAGNOSTIC_PERIOD_MS * 1000u, // 现有诊断周期，从 ms 转为 μs。
            .budget_us = ROBOT_DIAGNOSTIC_PERIOD_MS * 1000u, // 保留现有单轮耗时告警所用的预算。
        },
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 使用 FreeRTOS 精确表示这一间隔。
    },
    [ROBOT_PERIODIC_INS] = {
        .timing = {
            .period_us = ROBOT_INS_PERIOD_MS * 1000u, // 保留原姿态名义间隔，单位 μs。
            .budget_us = ROBOT_INS_PERIOD_MS * 1000u, // 保留原姿态耗时告警阈值。
        },
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 姿态任务使用 RTOS 刻度。
    },
    [ROBOT_PERIODIC_COMM] = {
        .timing = {
            .period_us = ROBOT_COMM_PERIOD_MS * 1000u, // 保留原通信名义间隔，单位 μs。
            .budget_us = ROBOT_COMM_PERIOD_MS * 1000u, // 保留原通信耗时告警阈值。
        },
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 通信派发使用 RTOS 刻度。
    },
    [ROBOT_PERIODIC_DAEMON] = {
        .timing = {
            .period_us = ROBOT_DAEMON_PERIOD_MS * 1000u, // 保留原设备维护名义间隔，单位 μs。
            .budget_us = ROBOT_DAEMON_PERIOD_MS * 1000u, // 保留原设备维护耗时告警阈值。
        },
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 设备维护使用 RTOS 刻度。
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

const PlatformPeriodicConfig_t *RobotGetTaskPeriod(RobotPeriodicTask_t task)
{
    if ((unsigned)task >= ROBOT_PERIODIC_COUNT) return NULL; // 越界时不读取表外数据。
    return &task_periods[task].timing;
}

#if MC02_TASK_MOTOR_SPLIT
static MotorTaskPeriods_t motor_task_periods; // 统一放行前写入，运行期只读。

PlatformPeriodicResult_t RobotPrepareMotorTaskPeriods(const MotorTaskPeriods_t *periods)
{
    const PlatformPeriodicCapabilities_t capabilities = PlatformTaskPeriodicCapabilities(); // 使用实际 RTOS 分辨率。
    const PlatformPeriodicConfig_t *dispatch = RobotGetTaskPeriod(ROBOT_PERIODIC_COMM); // 接收仍按现有周期派发。
    MotorTaskSchedule_t schedule; // 检查生产用的阶段计时，任务稍后在自己的上下文建立第一拍。
    const PlatformPeriodicResult_t result = MotorTaskScheduleInit(&schedule, periods, dispatch->period_us,
        capabilities.rtos_quantum_us, 0u);
    if (result == PLATFORM_PERIODIC_READY) motor_task_periods = *periods; // 两项通过后才保存。
    return result;
}

const MotorTaskPeriods_t *RobotGetMotorTaskPeriods(void)
{
    return motor_task_periods.compute_us != 0u ? &motor_task_periods : NULL;
}
#endif

#if MC02_TASK_FIXED_REGULAR_PERIODS
PlatformPeriodicResult_t RobotValidateTaskPeriods(const RobotModulePlan_t *modules, RobotPeriodicTask_t *failed)
{
    PlatformPeriodicRequest_t enabled[ROBOT_PERIODIC_COUNT];
    RobotPeriodicTask_t ids[ROBOT_PERIODIC_COUNT];
    size_t count = 0u; // 只为本次配置会创建的任务检查时钟。
    for (size_t i = 0u; i < ROBOT_PERIODIC_COUNT; ++i)
    {
        if (i == ROBOT_PERIODIC_INS && modules != NULL &&
            (modules->enabled & ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)) == 0u) continue; // 无姿态模块时略过。
#if !MC02_FEATURE_COMM_DISPATCH
        if (i == ROBOT_PERIODIC_COMM) continue; // 此构建不创建通信派发任务。
#endif
        enabled[count] = task_periods[i]; // 将实际请求交给平台接口。
        ids[count++] = (RobotPeriodicTask_t)i; // 保留原任务编号，不能把压缩后的索引直接用于诊断。
    }
    const PlatformPeriodicCapabilities_t capabilities = PlatformTaskPeriodicCapabilities(); // 读取实际唤醒能力。
    uint32_t hardware_mask; // 计划函数需完成整个分配；诊断项的后端已由配置指定。
    size_t failed_index = 0u; // 固定表首项用于报告通用配置错误。
    const PlatformPeriodicResult_t result = PlatformPeriodicPlan(enabled, count,
        &capabilities, &hardware_mask, &failed_index); // 依据各任务指定的时钟检查精度和容量。
    if (result != PLATFORM_PERIODIC_READY && failed != NULL)
        *failed = ids[failed_index]; // 报告真正失败的任务。
    return result;
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
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = ROBOT_INS_PERIOD_MS, .deadline_ms = 5u, .startup_grace_ms = 100u}, // 姿态周期。
#if MC02_FEATURE_COMM_DISPATCH
    {.id = TASK_MONITOR_MOTOR, .name = "MOTOR", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = ROBOT_COMM_PERIOD_MS, .deadline_ms = 5u, .startup_grace_ms = 100u}, // 电机及通信派发。
#endif
    {.id = TASK_MONITOR_ROBOT, .name = "ROBOT", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = ROBOT_DIAGNOSTIC_PERIOD_MS,
     .deadline_ms = 20u, .startup_grace_ms = 100u}, // 整车任务，监控沿用以毫秒表达的期限。
    {.id = TASK_MONITOR_DAEMON, .name = "DAEMON", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = ROBOT_DAEMON_PERIOD_MS, .deadline_ms = 30u, .startup_grace_ms = 100u}, // 设备及任务监控。
    {.id = TASK_MONITOR_UART_TX, .name = "UART_TX", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 30u, .startup_grace_ms = 100u}, // 串口发送。
};

#if MC02_TASK_CONTROL
static RobotControlTaskConfig_t control_config[ROBOT_CONTROL_TASK_COUNT]; // 放行后保持只读。
static uint32_t prepared_tasks; // 只有监控接受的任务才允许创建。

static const struct {
    const char *name;
    RobotModule_t module;
    TaskMonitor_Id_e monitor;
} control_monitor[ROBOT_CONTROL_TASK_COUNT] = {
#if MC02_TASK_CHASSIS
    [ROBOT_CONTROL_TASK_CHASSIS] = {"CHASSIS", ROBOT_MODULE_CHASSIS, TASK_MONITOR_CHASSIS}, // 底盘任务独立报告进度。
#endif
#if MC02_TASK_GIMBAL
    [ROBOT_CONTROL_TASK_GIMBAL] = {"GIMBAL", ROBOT_MODULE_GIMBAL, TASK_MONITOR_GIMBAL}, // 反馈发布及控制共用云台监控。
#endif
#if MC02_TASK_SHOOTER
    [ROBOT_CONTROL_TASK_SHOOTER] = {"SHOOTER", ROBOT_MODULE_SHOOTER, TASK_MONITOR_SHOOTER}, // 发射模块由自己的任务报告进度。
#endif
};

// 监控以整毫秒显示名义周期；实际任务保持原来的微秒拍点。
static uint32_t ControlMonitorPeriodMs(uint32_t period_us)
{
    return period_us / 1000u + (period_us % 1000u != 0u); // 避免亚毫秒周期在监控中被记为 0。
}

static uint8_t ControlMonitorConfigValid(const RobotControlTaskConfig_t *config)
{
    return config != NULL && config->deadline_ms != 0u &&
        ControlMonitorPeriodMs(config->timing.period_us) <= config->deadline_ms && config->deadline_ms < (uint32_t)INT32_MAX &&
        config->startup_grace_ms != 0u && config->startup_grace_ms < (uint32_t)INT32_MAX &&
        config->stack_words != 0u && config->stack_words <= UINT16_MAX; // 此处检查毫秒监控与栈，平台另检查拍点和预算。
}

// 用同一张平台资源表检查本次启用的机构，保留实际时钟来源供运行阶段使用。
static RobotControlTimingFailure_t ResolveControlTasks(const RobotModulePlan_t *plan,
    const RobotControlTaskConfig_t *control, PlatformPeriodicSource_t sources[ROBOT_CONTROL_TASK_COUNT], uint32_t *enabled)
{
    PlatformPeriodicRequest_t requests[ROBOT_CONTROL_TASK_COUNT]; // 合并所有需要平台时钟的请求。
    RobotControlTaskId_t tasks[ROBOT_CONTROL_TASK_COUNT]; // 用于把压缩后的下标换回任务编号。
    size_t count = 0u;
    *enabled = 0u; // 未启用的任务不占监控和定时器通道。
    for (unsigned i = 0u; i < ROBOT_CONTROL_TASK_COUNT; ++i)
    {
        if (control_monitor[i].name == NULL || !(plan->enabled & ROBOT_MODULE_BIT(control_monitor[i].module)))
            continue; // 同时检查编译选择和本次车型模块计划。
        if (control == NULL || !ControlMonitorConfigValid(&control[i]))
            return (RobotControlTimingFailure_t){(RobotControlTaskId_t)i, PLATFORM_PERIODIC_CONFIG}; // 缺少栈或监控参数时拒绝整组。
        requests[count] = (PlatformPeriodicRequest_t){.timing = control[i].timing, .source = control[i].source}; // 保留微秒值和允许的来源。
        tasks[count++] = (RobotControlTaskId_t)i; // 保留原编号供结果定位。
        *enabled |= UINT32_C(1) << i; // 记录本次计划的任务。
    }
    if (count == 0u) return (RobotControlTimingFailure_t){ROBOT_CONTROL_TASK_COUNT, PLATFORM_PERIODIC_READY};
    const PlatformPeriodicCapabilities_t capabilities = PlatformTaskPeriodicCapabilities(); // 所有请求共享同一份平台容量。
    uint32_t hardware_mask; // 只在平台检查通过后读取。
    size_t failed_index = 0u; // 平台失败时返回对应请求下标。
    const PlatformPeriodicResult_t result = PlatformPeriodicPlan(requests, count, &capabilities, &hardware_mask, &failed_index);
    if (result != PLATFORM_PERIODIC_READY)
        return (RobotControlTimingFailure_t){tasks[failed_index], result}; // 报告具体机构的时间或资源错误。
    for (size_t i = 0u; i < count; ++i)
        sources[tasks[i]] = (hardware_mask & (UINT32_C(1) << i))
            ? PLATFORM_PERIODIC_SOURCE_HIGH_RES : PLATFORM_PERIODIC_SOURCE_RTOS; // 保存方案选中的来源，不在运行中重新选择。
    return (RobotControlTimingFailure_t){ROBOT_CONTROL_TASK_COUNT, PLATFORM_PERIODIC_READY};
}

const RobotControlTaskConfig_t *RobotGetControlTaskConfig(RobotControlTaskId_t task)
{
    if ((unsigned)task >= ROBOT_CONTROL_TASK_COUNT || !(prepared_tasks & (UINT32_C(1) << task))) return NULL;
    return &control_config[task];
}
#endif

TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan,
    const RobotControlTaskConfig_t *control, RobotControlTimingFailure_t *failure)
{
    if (failure != NULL) *failure = (RobotControlTimingFailure_t){ROBOT_CONTROL_TASK_COUNT, PLATFORM_PERIODIC_READY};
#if MC02_TASK_CONTROL
    TaskMonitorTaskConfig_t selected[sizeof(robot_task_monitor_config) /
                                      sizeof(robot_task_monitor_config[0]) + ROBOT_CONTROL_TASK_COUNT]; // 给独立控制任务留出容量。
#else
    (void)control; // 此构建不会启动独立控制任务。
    TaskMonitorTaskConfig_t selected[sizeof(robot_task_monitor_config) /
                                      sizeof(robot_task_monitor_config[0])]; // 使用固定容量。
#endif
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
#if MC02_TASK_CONTROL
    PlatformPeriodicSource_t sources[ROBOT_CONTROL_TASK_COUNT]; // 本次预检得到的时钟来源。
    uint32_t enabled; // 成功时表示所选控制任务。
    const RobotControlTimingFailure_t timing = ResolveControlTasks(plan, control, sources, &enabled);
    if (timing.reason != PLATFORM_PERIODIC_READY)
    {
        if (failure != NULL) *failure = timing; // 不发布部分监控或部分时钟方案。
        return TASK_MONITOR_RESULT_BAD_CONFIGURATION;
    }
    for (unsigned i = 0u; i < ROBOT_CONTROL_TASK_COUNT; ++i)
    {
        if (!(enabled & (UINT32_C(1) << i))) continue;
        selected[count++] = (TaskMonitorTaskConfig_t){
            .id = control_monitor[i].monitor, // 由对应任务自己报告进度。
            .name = control_monitor[i].name, // 用于健康查询和故障日志。
            .mode = TASK_MONITOR_MODE_PERIODIC, // 预期周期性取得进度。
            .role = TASK_MONITOR_ROLE_CORE, // 当前运动模块按核心任务关断输出。
            .period_ms = ControlMonitorPeriodMs(control[i].timing.period_us), // 仅监控显示向上取整，拍点仍用原单位。
            .deadline_ms = control[i].deadline_ms, // 使用车型确认的完成期限。
            .startup_grace_ms = control[i].startup_grace_ms, // 从统一放行开始计算宽限。
        };
    }
    const TaskMonitor_Result_e result = TaskMonitorInitConfigured(selected, count); // 集中发布监控配置。
    if (result == TASK_MONITOR_RESULT_OK)
    {
        for (unsigned i = 0u; i < ROBOT_CONTROL_TASK_COUNT; ++i)
        {
            if (!(enabled & (UINT32_C(1) << i))) continue;
            control_config[i] = control[i]; // 保存周期、监控与栈配置。
            control_config[i].source = sources[i]; // 保存预检选中的实际来源。
        }
        prepared_tasks = enabled; // 整表成功后才发布任务配置。
    }
    return result;
#else
    return TaskMonitorInitConfigured(selected, count); // 监控模块复制配置，局部数组可释放。
#endif
}
