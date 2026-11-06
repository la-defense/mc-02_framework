/** @file task_monitor.c
 * @brief 保存任务工作进度、检查固定期限，并按核心／辅助角色报告故障。
 * @note 配置和状态均使用固定数组；普通任务之外不调用本模块的进度接口。
 */
#include "task_monitor.h"

#include "robot_safety.h"
#include "bsp_log.h"
#include "../../bsp/os/platform_irq.h"
#include "../../bsp/os/platform_sync.h"
#include "../../bsp/os/platform_os.h"
#include "../../bsp/os/platform_safety.h"

#include <limits.h>
#include <string.h>

typedef struct
{
    TaskMonitorTaskConfig_t config;
    uint32_t last_feed_ms;
    uint32_t job_started_ms;
    uint32_t last_progress_ms;
    uint32_t last_completed_ms;
    uint32_t waiting_since_ms;
    uint32_t begin_count;
    uint32_t progress_count;
    uint32_t complete_count;
    uint32_t progress_value;
    TaskMonitor_State_e state;
    TaskMonitor_FaultReason_e fault_reason;
    TaskMonitor_WaitReason_e wait_reason;
    uint8_t started;
    uint8_t work_active;
    uint8_t has_completed;
    uint8_t alive;
} TaskMonitorItem_t;

typedef struct
{
    uint32_t started_ms;
    uint8_t active;
} TaskMonitor_LongOperationWindow_t;

typedef struct
{
    TaskMonitor_Id_e id;
    const char *name;
    TaskMonitor_Role_e role;
    uint32_t deadline_ms;
    uint32_t dependency_mask;
    TaskMonitor_DegradeCallback_t callback;
    void *callback_context;
} TaskMonitorFaultAction_t;

static const TaskMonitorTaskConfig_t default_configs[TASK_MONITOR_COUNT] = {
    {.id = TASK_MONITOR_INS, .name = "INS", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u},
    {.id = TASK_MONITOR_MOTOR, .name = "MOTOR", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 5u},
    {.id = TASK_MONITOR_ROBOT, .name = "ROBOT", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 5u, .deadline_ms = 20u},
    {.id = TASK_MONITOR_DAEMON, .name = "DAEMON", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 10u, .deadline_ms = 30u},
    {.id = TASK_MONITOR_UART_TX, .name = "UART_TX", .mode = TASK_MONITOR_MODE_PERIODIC,
     .role = TASK_MONITOR_ROLE_CORE, .period_ms = 1u, .deadline_ms = 30u},
};

static TaskMonitorItem_t monitor_items[TASK_MONITOR_MAX_TASKS];
static uint32_t monitor_configured_count;
static uint32_t monitor_started_ms;
static uint8_t monitor_initialized;
static uint8_t monitor_runtime_started;
static uint8_t monitor_runtime_deferred; // 调度器运行后仍等待协调者放行周期任务。
static uint8_t core_fault_latched;
static uint8_t long_operation_timeout_latched;
static uint8_t long_operation_fault_latched;
static uint32_t monitor_tick_count;
#define STARTUP_CALIBRATION_FEED_INTERVAL_MS 100u // 成功采样后最多每 100 ms 重载一次。
static uint32_t startup_sample_sequence; // 全工程唯一 INS 启动标定的有效样本编号。
static uint32_t startup_last_feed_ms; // 上次因有效启动样本喂狗的时间。
static TaskMonitor_LongOperationWindow_t
    long_operation_windows[TASK_MONITOR_MAX_TASKS][TASK_MONITOR_LONG_OPERATION_COUNT];

static void TaskMonitorNoteRuntimeStartLocked(uint32_t now, uint8_t scheduler_running)
{
    if (monitor_runtime_started == 0u && monitor_runtime_deferred == 0u && scheduler_running != 0u)
    {
        // 初始化可能早于调度器很久；启动宽限从任务真正开始运行时计时。
        monitor_started_ms = now;
        monitor_runtime_started = 1u;
    }
}

// 只在更新固定数组时关中断，日志和用户回调都在恢复中断后执行。
static uint32_t TaskMonitorEnterCritical(void)
{
    return PlatformIrqSaveAndDisable(); // 保留调用前的屏蔽状态，供同一上下文恢复。
}

static void TaskMonitorExitCritical(uint32_t previous)
{
    PlatformIrqRestore(previous); // 恢复进入前的状态，避免误开原本已屏蔽的中断。
}

static TaskMonitorItem_t *TaskMonitorFindItem(TaskMonitor_Id_e id)
{
    if ((uint32_t)id >= TASK_MONITOR_MAX_TASKS)
        return NULL;

    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        if (monitor_items[index].config.id == id)
            return &monitor_items[index];
    }
    return NULL;
}

static const TaskMonitorItem_t *TaskMonitorFindItemConst(TaskMonitor_Id_e id)
{
    return TaskMonitorFindItem(id);
}

static uint32_t TaskMonitorLongOperationLimitMs(TaskMonitor_LongOperation_e operation)
{
    switch (operation)
    {
    case TASK_MONITOR_LONG_OPERATION_CALIBRATION:
        return 15000u;
    case TASK_MONITOR_LONG_OPERATION_FLASH:
        return 8000u;
    default:
        return 0u;
    }
}

static uint8_t TaskMonitorHasActiveLongOperation(TaskMonitor_Id_e id, uint32_t now)
{
    uint8_t active = 0u;
    for (uint32_t operation = 0u; operation < TASK_MONITOR_LONG_OPERATION_COUNT; ++operation)
    {
        const TaskMonitor_LongOperationWindow_t *window =
            &long_operation_windows[TASK_MONITOR_INS][operation];
        if (window->active == 0u)
            continue;

        const uint32_t limit_ms = TaskMonitorLongOperationLimitMs((TaskMonitor_LongOperation_e)operation);
        if ((uint32_t)(now - window->started_ms) >= limit_ms)
            long_operation_timeout_latched = 1u;
        else if (operation == TASK_MONITOR_LONG_OPERATION_FLASH || id == TASK_MONITOR_INS)
            active = 1u;
    }
    return (active != 0u && long_operation_timeout_latched == 0u) ? 1u : 0u;
}

static uint8_t TaskMonitorHasAnyActiveLongOperation(void)
{
    for (uint32_t operation = 0u; operation < TASK_MONITOR_LONG_OPERATION_COUNT; ++operation)
    {
        if (long_operation_windows[TASK_MONITOR_INS][operation].active != 0u)
            return 1u;
    }
    return 0u;
}

static TaskMonitor_FaultReason_e TaskMonitorExpiredReason(const TaskMonitorItem_t *item,
                                                          uint32_t now,
                                                          uint32_t monitor_start_ms)
{
    if (item->fault_reason != TASK_MONITOR_FAULT_NONE ||
        TaskMonitorHasActiveLongOperation(item->config.id, now) != 0u)
        return TASK_MONITOR_FAULT_NONE;

    if (monitor_runtime_deferred != 0u)
        return TASK_MONITOR_FAULT_NONE; // 周期任务尚未放行，不计算其首次运行期限。

    if (item->started == 0u)
        return ((uint32_t)(now - monitor_start_ms) > item->config.startup_grace_ms)
                   ? TASK_MONITOR_FAULT_STARTUP_DEADLINE
                   : TASK_MONITOR_FAULT_NONE;

    if (item->config.mode == TASK_MONITOR_MODE_EVENT)
    {
        if (item->work_active != 0u &&
            (uint32_t)(now - item->job_started_ms) > item->config.deadline_ms)
            return TASK_MONITOR_FAULT_WORK_DEADLINE;
        return TASK_MONITOR_FAULT_NONE;
    }

    if (item->work_active != 0u)
        return ((uint32_t)(now - item->job_started_ms) > item->config.deadline_ms)
                   ? TASK_MONITOR_FAULT_WORK_DEADLINE
                   : TASK_MONITOR_FAULT_NONE;

    if (item->has_completed != 0u)
        return ((uint32_t)(now - item->last_completed_ms) > item->config.deadline_ms)
                   ? TASK_MONITOR_FAULT_PERIOD_DEADLINE
                   : TASK_MONITOR_FAULT_NONE;

    return ((uint32_t)(now - item->last_feed_ms) > item->config.deadline_ms)
               ? TASK_MONITOR_FAULT_PERIOD_DEADLINE
               : TASK_MONITOR_FAULT_NONE;
}

static uint8_t TaskMonitorLatchFaultLocked(TaskMonitorItem_t *item,
                                           TaskMonitor_FaultReason_e reason,
                                           TaskMonitorFaultAction_t *action)
{
    if (item->fault_reason != TASK_MONITOR_FAULT_NONE)
        return 0u;

    item->fault_reason = reason;
    item->alive = 0u;
    item->work_active = 0u;
    item->state = TASK_MONITOR_STATE_FAULTED;
    if (item->config.role == TASK_MONITOR_ROLE_CORE)
    {
        core_fault_latched = 1u;
        if (TaskMonitorHasAnyActiveLongOperation() != 0u)
            long_operation_fault_latched = 1u;
    }

    action->id = item->config.id;
    action->name = item->config.name;
    action->role = item->config.role;
    action->deadline_ms = item->config.deadline_ms;
    action->dependency_mask = item->config.dependency_mask;
    action->callback = item->config.degrade_callback;
    action->callback_context = item->config.callback_context;
    return 1u;
}

static void TaskMonitorPublishFaultActions(const TaskMonitorFaultAction_t *actions,
                                          uint32_t count,
                                          uint8_t core_healthy)
{
    for (uint32_t index = 0u; index < count; ++index)
    {
        const TaskMonitorFaultAction_t *action = &actions[index];
        if (action->role == TASK_MONITOR_ROLE_CORE)
        {
            // 先保存任务期限原因，之后IWDG复位才能计入有限恢复额度。
            PlatformRecordCoreTaskFault(); // 记录后重启才能识别任务故障。
            PlatformSafetyLatchOutputsOff(); // 立即关断并保持中断屏蔽。
        }
        else if (action->callback != NULL)
            action->callback(action->id, action->dependency_mask, action->callback_context);

        // 先关断核心输出，再把固定任务名和期限写入日志事件。
        BSP_LOG_EVENT_2(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_01,
                        BSP_LOG_FIELD_CSTR_LIMIT((action->name), 48u),
                        BSP_LOG_FIELD_U32(action->deadline_ms));
    }

    RobotSafetySetTaskHealthy(core_healthy);
}

static uint8_t TaskMonitorValidateConfig(const TaskMonitorTaskConfig_t *configs,
                                         uint32_t count)
{
    uint8_t seen[TASK_MONITOR_MAX_TASKS] = {0u};
    if (configs == NULL || count == 0u || count > TASK_MONITOR_MAX_TASKS)
        return 0u;

    for (uint32_t index = 0u; index < count; ++index)
    {
        const TaskMonitorTaskConfig_t *config = &configs[index];
        if ((uint32_t)config->id >= TASK_MONITOR_MAX_TASKS || config->name == NULL ||
            config->deadline_ms == 0u || config->deadline_ms >= (uint32_t)INT32_MAX ||
            config->startup_grace_ms >= (uint32_t)INT32_MAX ||
            (uint32_t)config->mode > TASK_MONITOR_MODE_EVENT ||
            (uint32_t)config->role > TASK_MONITOR_ROLE_AUXILIARY)
            return 0u;
        if ((config->mode == TASK_MONITOR_MODE_PERIODIC &&
             (config->period_ms == 0u || config->period_ms > config->deadline_ms)) ||
            (config->mode == TASK_MONITOR_MODE_EVENT && config->period_ms != 0u))
            return 0u;
        if (config->role == TASK_MONITOR_ROLE_AUXILIARY && config->dependency_mask == 0u)
            return 0u;
        if (seen[(uint32_t)config->id] != 0u)
            return 0u;
        seen[(uint32_t)config->id] = 1u;
    }
    return 1u;
}

TaskMonitor_Result_e TaskMonitorInitConfigured(const TaskMonitorTaskConfig_t *configs,
                                               uint32_t count)
{
    if (PlatformIrqIsInHandler() != 0u)
        return TASK_MONITOR_RESULT_CONTEXT;
    // 运行后重新初始化会清除已锁存的故障，必须在创建任务前完成配置。
    if (PlatformSchedulerIsRunning() != 0u)
        return TASK_MONITOR_RESULT_SCHEDULER_RUNNING;
    if (configs == NULL || count == 0u || count > TASK_MONITOR_MAX_TASKS)
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    if (TaskMonitorValidateConfig(configs, count) == 0u)
        return TASK_MONITOR_RESULT_BAD_CONFIGURATION;

    TaskMonitorItem_t candidate[TASK_MONITOR_MAX_TASKS] = {0};
    const uint32_t now = PlatformTimeNowMs();
    for (uint32_t index = 0u; index < count; ++index)
    {
        candidate[index].config = configs[index];
        candidate[index].last_feed_ms = now;
        candidate[index].last_completed_ms = now;
        candidate[index].alive = 1u;
        candidate[index].state = TASK_MONITOR_STATE_NOT_STARTED;
    }

    const uint32_t previous = TaskMonitorEnterCritical();
    memset(monitor_items, 0, sizeof(monitor_items));
    memcpy(monitor_items, candidate, count * sizeof(candidate[0]));
    memset(long_operation_windows, 0, sizeof(long_operation_windows));
    monitor_configured_count = count;
    monitor_started_ms = now;
    core_fault_latched = 0u;
    monitor_runtime_started = 0u;
    monitor_runtime_deferred = 0u; // 默认沿用原有调度器启动即开始监控的行为。
    long_operation_timeout_latched = 0u;
    long_operation_fault_latched = 0u;
    monitor_tick_count = 0u;
    monitor_initialized = 1u;
    TaskMonitorExitCritical(previous);
    return TASK_MONITOR_RESULT_OK;
}

uint8_t TaskMonitorInit(void)
{
    return (uint8_t)(TaskMonitorInitConfigured(default_configs, TASK_MONITOR_COUNT) ==
                     TASK_MONITOR_RESULT_OK);
}

TaskMonitor_Result_e TaskMonitorDeferRuntime(void)
{
    if (PlatformIrqIsInHandler() != 0u) return TASK_MONITOR_RESULT_CONTEXT; // 中断中不能变更监控阶段。
    if (PlatformSchedulerIsRunning() != 0u) return TASK_MONITOR_RESULT_SCHEDULER_RUNNING; // 只能在调度器启动前声明。
    const uint32_t previous = TaskMonitorEnterCritical(); // 与任务状态更新使用相同短临界区。
    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_OK; // 先假定可以推迟。
    if (monitor_initialized == 0u) result = TASK_MONITOR_RESULT_NOT_INITIALIZED; // 未配置不能推迟。
    else if (monitor_runtime_started != 0u || monitor_runtime_deferred != 0u ||
             TaskMonitorHasAnyActiveLongOperation() != 0u || core_fault_latched != 0u ||
             long_operation_timeout_latched != 0u || long_operation_fault_latched != 0u)
        result = TASK_MONITOR_RESULT_BUSY; // 已开始、已推迟或故障时不重置监控。
    else
    {
        for (uint32_t index = 0u; index < monitor_configured_count; ++index)
            if (monitor_items[index].started != 0u) result = TASK_MONITOR_RESULT_BUSY; // 已报告的任务不能被重新隐藏。
        if (result == TASK_MONITOR_RESULT_OK) monitor_runtime_deferred = 1u; // 所有检查通过后才改变状态。
    }
    TaskMonitorExitCritical(previous); // 恢复调用前中断状态。
    return result;
}

TaskMonitor_Result_e TaskMonitorRemoveDeferredAuxiliary(TaskMonitor_Id_e id)
{
    if (PlatformIrqIsInHandler()) return TASK_MONITOR_RESULT_CONTEXT; // 中断不能修改监控配置。
    if ((uint32_t)id >= TASK_MONITOR_MAX_TASKS) return TASK_MONITOR_RESULT_BAD_ARGUMENT; // 越界编号不访问数组。
    const uint8_t scheduler_running = PlatformSchedulerIsRunning(); // 区分启动前与协调阶段。
    const uint32_t previous = TaskMonitorEnterCritical(); // 与放行和故障状态更新同步。
    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_BAD_ARGUMENT; // 未找到任务时不改变配置。
    if (!monitor_initialized) result = TASK_MONITOR_RESULT_NOT_INITIALIZED; // 必须先完成配置。
    else if (monitor_runtime_started || (scheduler_running && !monitor_runtime_deferred))
        result = TASK_MONITOR_RESULT_NOT_RUNNING; // 运行期不能删除监控。
    else if (core_fault_latched || long_operation_timeout_latched || long_operation_fault_latched ||
             TaskMonitorHasAnyActiveLongOperation()) result = TASK_MONITOR_RESULT_BUSY; // 故障或长操作期间不能改表。
    else
    {
        for (uint32_t index = 0u; index < monitor_configured_count; ++index)
        {
            if (monitor_items[index].config.id != id) continue; // 跳过其他任务。
            TaskMonitorItem_t *item = &monitor_items[index]; // 找到要移除的配置。
            if (item->config.role != TASK_MONITOR_ROLE_AUXILIARY || item->started ||
                item->work_active || item->fault_reason != TASK_MONITOR_FAULT_NONE)
                result = TASK_MONITOR_RESULT_BUSY; // 核心任务或已运行任务不能被隐藏。
            else
            {
                for (uint32_t move = index + 1u; move < monitor_configured_count; ++move)
                    monitor_items[move - 1u] = monitor_items[move]; // 保持其他任务顺序和状态。
                --monitor_configured_count; // 只删除这一项。
                memset(&monitor_items[monitor_configured_count], 0, sizeof(monitor_items[0])); // 清除尾部残留。
                result = TASK_MONITOR_RESULT_OK; // 后续查询将报告该任务未配置。
            }
            break; // 任务编号唯一，找到后停止遍历。
        }
    }
    TaskMonitorExitCritical(previous); // 恢复调用前的中断状态。
    return result; // 由协调者决定是否继续启动。
}

TaskMonitor_Result_e TaskMonitorReleaseRuntime(void)
{
    if (PlatformIrqIsInHandler() != 0u) return TASK_MONITOR_RESULT_CONTEXT; // 必须由启动协调任务调用。
    if (PlatformSchedulerIsRunning() == 0u) return TASK_MONITOR_RESULT_NOT_RUNNING; // 调度器未运行时不能放行。
    const uint32_t now = PlatformTimeNowMs(); // 一次读取，所有周期任务共用同一放行时刻。
    const uint32_t previous = TaskMonitorEnterCritical(); // 防止任务同时读取未完成的状态。
    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_OK; // 先假定可以放行。
    if (monitor_initialized == 0u) result = TASK_MONITOR_RESULT_NOT_INITIALIZED; // 缺少配置不能放行。
    else if (monitor_runtime_deferred == 0u) result = TASK_MONITOR_RESULT_NOT_RUNNING; // 不能重复重置宽限。
    else if (core_fault_latched != 0u || long_operation_timeout_latched != 0u ||
             long_operation_fault_latched != 0u || TaskMonitorHasAnyActiveLongOperation() != 0u)
        result = TASK_MONITOR_RESULT_BUSY; // 故障或长操作尚未结束时保持阻止。
    else
    {
        monitor_started_ms = now; // 任务首次运行宽限从真实放行时刻开始。
        monitor_runtime_started = 1u; // 防止后续调用再次改写起点。
        monitor_runtime_deferred = 0u; // 最后发布运行阶段，允许任务报告进度。
    }
    TaskMonitorExitCritical(previous); // 完整发布后恢复中断。
    return result;
}

uint8_t TaskMonitorRuntimeReleased(void)
{
    const uint8_t scheduler_running = PlatformSchedulerIsRunning(); // 查询调度器是否已经运行。
    const uint32_t previous = TaskMonitorEnterCritical(); // 与放行操作使用同一短临界区。
    const uint8_t released = (uint8_t)(scheduler_running && monitor_initialized &&
                                       !monitor_runtime_deferred); // 尚未初始化或仍在协调时保持关闭。
    TaskMonitorExitCritical(previous); // 恢复调用前的中断状态。
    return released; // 查询不会改变期限或故障状态。
}

TaskMonitor_Result_e TaskMonitorBeginWork(TaskMonitor_Id_e id)
{
    if (PlatformIrqIsInHandler() != 0u)
        return TASK_MONITOR_RESULT_CONTEXT;

    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_OK;
    TaskMonitorFaultAction_t action = {0};
    uint8_t publish_fault = 0u;
    uint8_t core_healthy = 1u;
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    TaskMonitorItem_t *item = TaskMonitorFindItem(id);
    if (monitor_initialized == 0u)
        result = TASK_MONITOR_RESULT_NOT_INITIALIZED;
    else if (monitor_runtime_deferred != 0u)
        result = TASK_MONITOR_RESULT_NOT_RUNNING; // 协调者尚未放行周期任务。
    else if (item == NULL)
        result = TASK_MONITOR_RESULT_BAD_ARGUMENT;
    else if (core_fault_latched != 0u && item->config.role == TASK_MONITOR_ROLE_CORE)
        result = TASK_MONITOR_RESULT_BUSY;
    else if (item->fault_reason != TASK_MONITOR_FAULT_NONE || item->work_active != 0u)
        result = TASK_MONITOR_RESULT_BUSY;
    else
    {
        const TaskMonitor_FaultReason_e expired =
            TaskMonitorExpiredReason(item, now, monitor_started_ms);
        if (expired != TASK_MONITOR_FAULT_NONE)
        {
            publish_fault = TaskMonitorLatchFaultLocked(item, expired, &action);
            result = TASK_MONITOR_RESULT_BUSY;
        }
        else
        {
            item->started = 1u;
            item->work_active = 1u;
            item->job_started_ms = now;
            item->last_progress_ms = now;
            item->progress_value = 0u;
            item->wait_reason = TASK_MONITOR_WAIT_NONE;
            item->state = TASK_MONITOR_STATE_RUNNING;
            item->alive = 1u;
            if (item->begin_count != UINT32_MAX)
                item->begin_count++;
        }
    }
    if (core_fault_latched != 0u)
        core_healthy = 0u;
    TaskMonitorExitCritical(previous);

    if (publish_fault != 0u)
        TaskMonitorPublishFaultActions(&action, 1u, core_healthy);
    return result;
}

TaskMonitor_Result_e TaskMonitorReportProgress(TaskMonitor_Id_e id,
                                               uint32_t progress_value)
{
    if (PlatformIrqIsInHandler() != 0u)
        return TASK_MONITOR_RESULT_CONTEXT;
    if (monitor_initialized == 0u)
        return TASK_MONITOR_RESULT_NOT_INITIALIZED;

    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    TaskMonitorItem_t *item = TaskMonitorFindItem(id);
    if (item == NULL)
    {
        TaskMonitorExitCritical(previous);
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    }
    if (item->work_active == 0u || item->fault_reason != TASK_MONITOR_FAULT_NONE)
    {
        TaskMonitorExitCritical(previous);
        return TASK_MONITOR_RESULT_NOT_RUNNING;
    }

    item->last_progress_ms = now;
    item->progress_value = progress_value;
    if (item->progress_count != UINT32_MAX)
        item->progress_count++;
    item->wait_reason = TASK_MONITOR_WAIT_NONE;
    item->state = TASK_MONITOR_STATE_RUNNING;
    TaskMonitorExitCritical(previous);
    return TASK_MONITOR_RESULT_OK;
}

TaskMonitor_Result_e TaskMonitorSetWaiting(TaskMonitor_Id_e id,
                                           TaskMonitor_WaitReason_e reason)
{
    if (PlatformIrqIsInHandler() != 0u)
        return TASK_MONITOR_RESULT_CONTEXT;
    if (monitor_initialized == 0u)
        return TASK_MONITOR_RESULT_NOT_INITIALIZED;
    if (reason == TASK_MONITOR_WAIT_NONE || (uint32_t)reason > TASK_MONITOR_WAIT_DELAY)
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;

    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    if (monitor_runtime_deferred != 0u)
    {
        TaskMonitorExitCritical(previous); // 启动等待不登记成尚未运行的周期任务。
        return TASK_MONITOR_RESULT_NOT_RUNNING;
    }
    TaskMonitorItem_t *item = TaskMonitorFindItem(id);
    if (item == NULL)
    {
        TaskMonitorExitCritical(previous);
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    }
    if (item->fault_reason != TASK_MONITOR_FAULT_NONE)
    {
        TaskMonitorExitCritical(previous);
        return TASK_MONITOR_RESULT_BUSY;
    }

    item->started = 1u;
    item->waiting_since_ms = now;
    item->wait_reason = reason;
    item->state = TASK_MONITOR_STATE_WAITING;
    TaskMonitorExitCritical(previous);
    return TASK_MONITOR_RESULT_OK;
}

TaskMonitor_Result_e TaskMonitorCompleteWork(TaskMonitor_Id_e id)
{
    if (PlatformIrqIsInHandler() != 0u)
        return TASK_MONITOR_RESULT_CONTEXT;
    if (monitor_initialized == 0u)
        return TASK_MONITOR_RESULT_NOT_INITIALIZED;

    TaskMonitorFaultAction_t action = {0};
    uint8_t publish_fault = 0u;
    uint8_t core_healthy = 1u;
    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_OK;
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    TaskMonitorItem_t *item = TaskMonitorFindItem(id);
    if (item == NULL)
        result = TASK_MONITOR_RESULT_BAD_ARGUMENT;
    else if (item->work_active == 0u)
        result = TASK_MONITOR_RESULT_NOT_RUNNING;
    else
    {
        const uint8_t expired =
            (uint32_t)(now - item->job_started_ms) > item->config.deadline_ms;
        item->work_active = 0u;
        item->has_completed = 1u;
        item->last_completed_ms = now;
        item->last_feed_ms = now;
        item->wait_reason = TASK_MONITOR_WAIT_NONE;
        if (item->complete_count != UINT32_MAX)
            item->complete_count++;

        if (expired != 0u)
        {
            publish_fault = TaskMonitorLatchFaultLocked(item,
                                                        TASK_MONITOR_FAULT_WORK_DEADLINE,
                                                        &action);
            result = TASK_MONITOR_RESULT_BUSY;
        }
        else
        {
            item->state = TASK_MONITOR_STATE_COMPLETE;
            item->alive = 1u;
        }
    }
    if (core_fault_latched != 0u)
        core_healthy = 0u;
    TaskMonitorExitCritical(previous);

    if (publish_fault != 0u)
        TaskMonitorPublishFaultActions(&action, 1u, core_healthy);
    return result;
}

TaskMonitor_Result_e TaskMonitorGetProgressState(TaskMonitor_Id_e id,
                                                 TaskMonitorProgressState_t *output)
{
    if (output == NULL)
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    if (monitor_initialized == 0u)
        return TASK_MONITOR_RESULT_NOT_INITIALIZED;

    const uint32_t previous = TaskMonitorEnterCritical();
    const TaskMonitorItem_t *item = TaskMonitorFindItemConst(id);
    if (item == NULL)
    {
        TaskMonitorExitCritical(previous);
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    }

    TaskMonitorProgressState_t candidate = {
        .mode = item->config.mode,
        .role = item->config.role,
        .state = item->state,
        .fault_reason = item->fault_reason,
        .wait_reason = item->wait_reason,
        .job_started_ms = item->job_started_ms,
        .last_progress_ms = item->last_progress_ms,
        .waiting_since_ms = item->waiting_since_ms,
        .last_completed_ms = item->last_completed_ms,
        .period_ms = item->config.period_ms,
        .deadline_ms = item->config.deadline_ms,
        .startup_grace_ms = item->config.startup_grace_ms,
        .begin_count = item->begin_count,
        .progress_count = item->progress_count,
        .complete_count = item->complete_count,
        .progress_value = item->progress_value,
        .dependency_mask = item->config.dependency_mask,
        .started = item->started,
        .work_active = item->work_active,
        .has_completed = item->has_completed,
        .configured = 1u,
    };
    TaskMonitorExitCritical(previous);
    *output = candidate;
    return TASK_MONITOR_RESULT_OK;
}

TaskMonitor_Result_e TaskMonitorGetDegradedDependencies(uint32_t *output)
{
    if (output == NULL)
        return TASK_MONITOR_RESULT_BAD_ARGUMENT;
    if (monitor_initialized == 0u)
        return TASK_MONITOR_RESULT_NOT_INITIALIZED;

    const uint32_t previous = TaskMonitorEnterCritical();
    uint32_t dependencies = 0u;
    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        const TaskMonitorItem_t *item = &monitor_items[index];
        if (item->config.role == TASK_MONITOR_ROLE_AUXILIARY &&
            item->fault_reason != TASK_MONITOR_FAULT_NONE)
            dependencies |= item->config.dependency_mask;
    }
    TaskMonitorExitCritical(previous);
    *output = dependencies;
    return TASK_MONITOR_RESULT_OK;
}

void TaskMonitorFeed(TaskMonitor_Id_e id)
{
    if (monitor_initialized == 0u)
        return;

    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    TaskMonitorItem_t *item = TaskMonitorFindItem(id);
    if (monitor_runtime_deferred == 0u && item != NULL && item->fault_reason == TASK_MONITOR_FAULT_NONE)
    {
        item->started = 1u;
        item->last_feed_ms = now;
        item->last_progress_ms = now;
        item->state = TASK_MONITOR_STATE_RUNNING;
        item->alive = 1u;
    }
    TaskMonitorExitCritical(previous);
}

void TaskMonitorDelayMs(TaskMonitor_Id_e id, uint32_t milliseconds)
{
    if ((uint32_t)id >= TASK_MONITOR_MAX_TASKS || milliseconds == 0u)
        return;

    if (PlatformSchedulerIsRunning() == 0u)
    {
        PlatformDelayBeforeSchedulerMs(milliseconds); // 保持调度器启动前原有的忙等。
        return;
    }

    // 分成 1 ms 等待，让其他任务运行；等待次数不会重置当前工作的开始时刻。
    while (milliseconds > 0u)
    {
        (void)TaskMonitorSetWaiting(id, TASK_MONITOR_WAIT_DELAY);
        PlatformTaskDelayMs(1u); // 使用原先 1 ms 的任务让出周期。
        TaskMonitorFeed(id);
        (void)TaskMonitorReportProgress(id, 0u);
        milliseconds--;
    }
}

void TaskMonitorTick(void)
{
    TaskMonitorFaultAction_t actions[TASK_MONITOR_MAX_TASKS] = {0};
    uint32_t action_count = 0u;
    uint8_t core_healthy = 1u;
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    if (monitor_initialized == 0u)
    {
        TaskMonitorExitCritical(previous);
        return;
    }
    if (monitor_runtime_deferred != 0u)
    {
        (void)TaskMonitorHasActiveLongOperation(TASK_MONITOR_INS, now); // 启动期仍检查长操作固定期限。
        if (long_operation_timeout_latched != 0u && long_operation_fault_latched == 0u)
        {
            TaskMonitorItem_t *ins = TaskMonitorFindItem(TASK_MONITOR_INS); // 找到声明的标定来源。
            long_operation_fault_latched = 1u; // 超期只记录一次，不能重新申请放行。
            if (ins != NULL && TaskMonitorLatchFaultLocked(ins, TASK_MONITOR_FAULT_WORK_DEADLINE,
                                                            &actions[action_count]) != 0u)
                action_count++; // 退出临界区后执行关断及故障发布。
        }
        TaskMonitorExitCritical(previous); // 尚未放行时不检查周期期限，也不喂狗。
        if (action_count != 0u) TaskMonitorPublishFaultActions(actions, action_count, 0u); // 长操作超期立即失能。
        return;
    }
    monitor_tick_count++;

    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        TaskMonitorItem_t *item = &monitor_items[index];
        const TaskMonitor_FaultReason_e expired =
            TaskMonitorExpiredReason(item, now, monitor_started_ms);
        if (expired != TASK_MONITOR_FAULT_NONE &&
            TaskMonitorLatchFaultLocked(item, expired, &actions[action_count]) != 0u)
        {
            action_count++;
        }
        if (item->config.role == TASK_MONITOR_ROLE_CORE &&
            item->fault_reason != TASK_MONITOR_FAULT_NONE)
            core_healthy = 0u;
    }
    if (long_operation_timeout_latched != 0u && long_operation_fault_latched == 0u)
    {
        TaskMonitorItem_t *ins = TaskMonitorFindItem(TASK_MONITOR_INS);
        long_operation_fault_latched = 1u;
        if (ins != NULL &&
            TaskMonitorLatchFaultLocked(ins, TASK_MONITOR_FAULT_WORK_DEADLINE,
                                        &actions[action_count]) != 0u)
            action_count++;
    }
    if (long_operation_timeout_latched != 0u || long_operation_fault_latched != 0u ||
        core_fault_latched != 0u)
        core_healthy = 0u;
    TaskMonitorExitCritical(previous);

    TaskMonitorPublishFaultActions(actions, action_count, core_healthy);
    // 只有核心工作仍正常时才重载 IWDG；辅助服务故障保留为各自的降级状态。
    if (core_healthy != 0u)
        PlatformWatchdogFeed(); // 仅在既有核心检查通过后重载看门狗。
}

uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id,
                                      TaskMonitor_LongOperation_e operation)
{
    const uint32_t limit_ms = TaskMonitorLongOperationLimitMs(operation);
    if (monitor_initialized == 0u || id != TASK_MONITOR_INS ||
        (uint32_t)id >= TASK_MONITOR_MAX_TASKS || limit_ms == 0u ||
        core_fault_latched != 0u || long_operation_timeout_latched != 0u ||
        long_operation_fault_latched != 0u)
        return 0u;

    TaskMonitorFaultAction_t actions[TASK_MONITOR_MAX_TASKS] = {0};
    uint32_t action_count = 0u;
    uint8_t allowed = 1u;
    uint8_t core_healthy = 1u;
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    TaskMonitor_LongOperationWindow_t *window = &long_operation_windows[id][operation];
    if (window->active != 0u)
        allowed = 0u;

    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        TaskMonitorItem_t *item = &monitor_items[index];
        if (item->config.role != TASK_MONITOR_ROLE_CORE)
            continue;
        const uint8_t startup_calibration_grace =
            (operation == TASK_MONITOR_LONG_OPERATION_CALIBRATION &&
             monitor_tick_count == 0u && item->alive != 0u)
                ? 1u
                : 0u;
        const TaskMonitor_FaultReason_e expired =
            TaskMonitorExpiredReason(item, now, monitor_started_ms);
        if (expired != TASK_MONITOR_FAULT_NONE && startup_calibration_grace == 0u)
        {
            if (TaskMonitorLatchFaultLocked(item, expired, &actions[action_count]) != 0u)
                action_count++;
            allowed = 0u;
        }
        if (item->fault_reason != TASK_MONITOR_FAULT_NONE)
            allowed = 0u;
    }

    if (long_operation_timeout_latched || long_operation_fault_latched || core_fault_latched)
        allowed = 0u; // 检查其他窗口时可能刚发现超期，不能再开启新操作。
    if (allowed != 0u)
    {
        window->started_ms = now;
        if (operation == TASK_MONITOR_LONG_OPERATION_CALIBRATION)
        {
            startup_sample_sequence = 0u; // 新的标定窗口从第一份有效样本开始计数。
            startup_last_feed_ms = now; // 首个喂狗间隔从申请时开始。
        }
        window->active = 1u;
    }
    if (core_fault_latched != 0u || allowed == 0u)
        core_healthy = 0u;
    TaskMonitorExitCritical(previous);

    if (action_count != 0u)
        TaskMonitorPublishFaultActions(actions, action_count, core_healthy);
    else if (allowed == 0u)
        RobotSafetySetTaskHealthy(core_healthy);
    return allowed;
}

TaskMonitor_Result_e TaskMonitorStartupCalibrationSample(uint32_t sequence)
{
    if (PlatformIrqIsInHandler()) return TASK_MONITOR_RESULT_CONTEXT; // 中断不能代替协调任务报告采样。
    if (sequence == 0u) return TASK_MONITOR_RESULT_BAD_ARGUMENT; // 零不能代表已完成的样本。
    if (!PlatformSchedulerIsRunning()) return TASK_MONITOR_RESULT_NOT_RUNNING; // 调度器启动前不能当作协调任务进度。
    const uint32_t now = PlatformTimeNowMs(); // 每次只读取一次原有毫秒时基。
    const uint32_t previous = TaskMonitorEnterCritical(); // 与故障锁存和窗口结束同步。
    TaskMonitor_LongOperationWindow_t *window =
        &long_operation_windows[TASK_MONITOR_INS][TASK_MONITOR_LONG_OPERATION_CALIBRATION]; // 只检查启动标定窗口。
    TaskMonitor_Result_e result = TASK_MONITOR_RESULT_OK; // 先假定该样本可接受。
    uint8_t expired = 0u; // 记录是否需要在退出临界区后报告超时。
    if (!monitor_initialized || !monitor_runtime_deferred || !window->active)
        result = TASK_MONITOR_RESULT_NOT_RUNNING; // 普通运行阶段不改变现有喂狗方式。
    else if (core_fault_latched || long_operation_fault_latched)
        result = TASK_MONITOR_RESULT_BUSY; // 已锁存故障不能通过新样本恢复。
    else if (long_operation_timeout_latched)
    {
        expired = 1u; // 其他查询已发现超期时，也要发布尚未处理的故障。
        result = TASK_MONITOR_RESULT_BUSY; // 不允许超期后借样本喂狗。
    }
    else if ((uint32_t)(now - window->started_ms) >=
             TaskMonitorLongOperationLimitMs(TASK_MONITOR_LONG_OPERATION_CALIBRATION))
    {
        long_operation_timeout_latched = 1u; // 标定总时限固定，不因新样本延长。
        expired = 1u; // 临界区外沿用任务监控的关断和诊断路径。
        result = TASK_MONITOR_RESULT_BUSY; // 超期样本不能喂狗。
    }
    else if (sequence <= startup_sample_sequence)
        result = TASK_MONITOR_RESULT_BAD_ARGUMENT; // 重复或倒退的计数不能制造进度。
    else
    {
        startup_sample_sequence = sequence; // 保存已接受样本，阻止重复提交。
        if ((uint32_t)(now - startup_last_feed_ms) >= STARTUP_CALIBRATION_FEED_INTERVAL_MS)
        {
            startup_last_feed_ms = now; // 只在时钟前进到间隔后更新时间。
            PlatformWatchdogFeed(); // 与故障检查原子衔接；底层仅执行有界寄存器写入。
        }
    }
    TaskMonitorExitCritical(previous); // 任何日志和故障回调都在恢复中断后执行。
    if (expired) TaskMonitorTick(); // 立即按已有核心超期规则记录并关断。
    return result; // 调用者据此继续标定或停止采样。
}

uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id,
                                    TaskMonitor_LongOperation_e operation)
{
    if (monitor_initialized == 0u || id != TASK_MONITOR_INS ||
        (uint32_t)id >= TASK_MONITOR_MAX_TASKS ||
        TaskMonitorLongOperationLimitMs(operation) == 0u)
        return 0u;

    TaskMonitorFaultAction_t actions[TASK_MONITOR_MAX_TASKS] = {0};
    uint32_t action_count = 0u;
    uint8_t within_limit = 0u;
    uint8_t core_healthy = 1u;
    const uint32_t now = PlatformTimeNowMs();
    const uint8_t scheduler_running = PlatformSchedulerIsRunning();
    const uint32_t previous = TaskMonitorEnterCritical();
    TaskMonitorNoteRuntimeStartLocked(now, scheduler_running);
    const uint8_t startup_deferred = monitor_runtime_deferred; // 启动标定结束不代表周期任务已健康运行。
    TaskMonitor_LongOperationWindow_t *window = &long_operation_windows[id][operation];
    if (window->active == 0u)
    {
        TaskMonitorExitCritical(previous);
        return 0u;
    }

    within_limit = (uint8_t)((uint32_t)(now - window->started_ms) <
                             TaskMonitorLongOperationLimitMs(operation));
    if (within_limit == 0u)
    {
        long_operation_timeout_latched = 1u;
        long_operation_fault_latched = 1u;
        TaskMonitorItem_t *ins = TaskMonitorFindItem(TASK_MONITOR_INS);
        if (ins != NULL && TaskMonitorLatchFaultLocked(ins,
                                                       TASK_MONITOR_FAULT_WORK_DEADLINE,
                                                       &actions[action_count]) != 0u)
            action_count++;
    }
    window->active = 0u;

    if (within_limit != 0u && long_operation_timeout_latched == 0u &&
        long_operation_fault_latched == 0u)
    {
        for (uint32_t index = 0u; index < monitor_configured_count; ++index)
        {
            TaskMonitorItem_t *item = &monitor_items[index];
            if (item->config.role != TASK_MONITOR_ROLE_CORE)
                continue;
            if (operation == TASK_MONITOR_LONG_OPERATION_FLASH ||
                item->config.id == TASK_MONITOR_INS)
            {
                item->last_feed_ms = now;
                if (item->work_active != 0u)
                    item->job_started_ms = now;
                else if (item->has_completed != 0u)
                    item->last_completed_ms = now;
            }
        }
    }

    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        TaskMonitorItem_t *item = &monitor_items[index];
        if (item->config.role != TASK_MONITOR_ROLE_CORE)
            continue;
        const TaskMonitor_FaultReason_e expired =
            TaskMonitorExpiredReason(item, now, monitor_started_ms);
        if (expired != TASK_MONITOR_FAULT_NONE &&
            TaskMonitorLatchFaultLocked(item, expired, &actions[action_count]) != 0u)
            action_count++;
        if (item->fault_reason != TASK_MONITOR_FAULT_NONE)
            core_healthy = 0u;
    }
    if (long_operation_timeout_latched != 0u || long_operation_fault_latched != 0u ||
        core_fault_latched != 0u)
        core_healthy = 0u;
    TaskMonitorExitCritical(previous);

    if (action_count != 0u)
        TaskMonitorPublishFaultActions(actions, action_count, core_healthy);
    else if (startup_deferred == 0u)
        RobotSafetySetTaskHealthy(core_healthy);
    return (uint8_t)(within_limit != 0u && core_healthy != 0u);
}

uint8_t TaskMonitorAllAlive(void)
{
    if (monitor_initialized == 0u)
        return 0u;

    const uint32_t previous = TaskMonitorEnterCritical();
    uint8_t alive = (uint8_t)(monitor_runtime_deferred == 0u && core_fault_latched == 0u &&
                              long_operation_timeout_latched == 0u &&
                              long_operation_fault_latched == 0u);
    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        const TaskMonitorItem_t *item = &monitor_items[index];
        if (item->config.role == TASK_MONITOR_ROLE_CORE && item->alive == 0u)
            alive = 0u;
    }
    TaskMonitorExitCritical(previous);
    return alive;
}

uint8_t TaskMonitorAllCoreHealthyNow(void)
{
    if (monitor_initialized == 0u)
        return 0u;

    const uint32_t now = PlatformTimeNowMs();
    const uint32_t previous = TaskMonitorEnterCritical();
    uint8_t has_core_task = 0u;
    uint8_t healthy = (uint8_t)(monitor_runtime_deferred == 0u && core_fault_latched == 0u &&
                                long_operation_timeout_latched == 0u &&
                                long_operation_fault_latched == 0u);
    for (uint32_t index = 0u; index < monitor_configured_count; ++index)
    {
        const TaskMonitorItem_t *item = &monitor_items[index];
        if (item->config.role != TASK_MONITOR_ROLE_CORE)
            continue;

        has_core_task = 1u;
        if (item->alive == 0u || item->fault_reason != TASK_MONITOR_FAULT_NONE ||
            TaskMonitorExpiredReason(item, now, monitor_started_ms) !=
                TASK_MONITOR_FAULT_NONE)
            healthy = 0u;
    }
    if (long_operation_timeout_latched != 0u || long_operation_fault_latched != 0u)
        healthy = 0u;
    TaskMonitorExitCritical(previous);
    return (has_core_task != 0u && healthy != 0u) ? 1u : 0u;
}

void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status)
{
    if (status == NULL || monitor_initialized == 0u)
        return;

    const uint32_t previous = TaskMonitorEnterCritical();
    const TaskMonitorItem_t *item = TaskMonitorFindItemConst(id);
    if (item != NULL)
    {
        status->name = item->config.name;
        status->alive = item->alive;
        status->fault = (uint8_t)(item->fault_reason != TASK_MONITOR_FAULT_NONE);
        status->timeout_ms = item->config.deadline_ms;
    }
    TaskMonitorExitCritical(previous);
}
