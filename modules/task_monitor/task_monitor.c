#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "robot_safety.h"
#include "bsp_log.h"
#include "main.h"
#include "bsp_safety.h"
#include <string.h>

typedef struct
{
    const char *name;
    uint32_t timeout_ms;
    uint32_t last_feed_ms;
    uint8_t alive;
    uint8_t fault_logged;
} TaskMonitorItem_t;

static TaskMonitorItem_t monitor_items[TASK_MONITOR_COUNT] = {
    [TASK_MONITOR_INS] = {.name = "INS", .timeout_ms = 5},
    [TASK_MONITOR_MOTOR] = {.name = "MOTOR", .timeout_ms = 5},
    [TASK_MONITOR_ROBOT] = {.name = "ROBOT", .timeout_ms = 20},
    [TASK_MONITOR_DAEMON] = {.name = "DAEMON", .timeout_ms = 30},
};

static uint8_t monitor_initialized = 0u;
static uint8_t long_operation_timeout_latched = 0u;
static uint32_t monitor_tick_count = 0u;

typedef struct
{
    uint32_t started_ms;
    uint8_t active;
} TaskMonitor_LongOperationWindow_t;

static TaskMonitor_LongOperationWindow_t long_operation_windows[TASK_MONITOR_COUNT][TASK_MONITOR_LONG_OPERATION_COUNT];

static uint32_t TaskMonitorLongOperationLimitMs(TaskMonitor_LongOperation_e operation)
{
    if ((uint32_t)operation >= TASK_MONITOR_LONG_OPERATION_COUNT)
        return 0u;

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
        /* H723 single-bank Flash stalls the whole core while erasing/programming.
           Calibration only blocks INS, so all other task deadlines stay live. */
        TaskMonitor_Id_e owner = TASK_MONITOR_INS;
        TaskMonitor_LongOperationWindow_t *window = &long_operation_windows[owner][operation];
        if (!window->active)
            continue;

        uint32_t limit_ms = TaskMonitorLongOperationLimitMs((TaskMonitor_LongOperation_e)operation);
        if ((uint32_t)(now - window->started_ms) >= limit_ms)
            long_operation_timeout_latched = 1u;
        else if (operation == TASK_MONITOR_LONG_OPERATION_FLASH || id == owner)
            active = 1u;
    }
    return (active && !long_operation_timeout_latched) ? 1u : 0u;
}

void TaskMonitorInit(void)
{
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        monitor_items[i].last_feed_ms = now;
        monitor_items[i].alive = 1;
        monitor_items[i].fault_logged = 0;
    }
    memset(long_operation_windows, 0, sizeof(long_operation_windows));
    long_operation_timeout_latched = 0u;
    monitor_tick_count = 0u;
    monitor_initialized = 1u;
}

void TaskMonitorFeed(TaskMonitor_Id_e id)
{
    if (id >= TASK_MONITOR_COUNT)
        return;

    monitor_items[id].last_feed_ms = HAL_GetTick();
    monitor_items[id].alive = 1;
}

void TaskMonitorTick(void)
{
    monitor_tick_count++;
    uint32_t now = HAL_GetTick();
    uint8_t all_alive = (long_operation_timeout_latched == 0u) ? 1u : 0u;

    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        TaskMonitorItem_t *item = &monitor_items[i];
        uint8_t long_operation_active = TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);
        if (!long_operation_active && (uint32_t)(now - item->last_feed_ms) > item->timeout_ms)
        {
            item->alive = 0;
            all_alive = 0;
            if (!item->fault_logged)
            {
                BSP_SafetyLatchOutputsOff();
                LOGERROR("[monitor] task %s timeout (%ums)", item->name, (unsigned)item->timeout_ms);
                item->fault_logged = 1;
            }
        }
        else if (long_operation_active)
        {
            item->alive = 1u;
        }
        else
        {
            item->alive = 1;
            item->fault_logged = 0;
        }
    }

    if (long_operation_timeout_latched)
        all_alive = 0u;

    RobotSafetySetTaskHealthy(all_alive);

    if (all_alive)
        BSP_WatchdogFeed();
    /* 任一关键任务超时: 不喂狗, 由 IWDG 复位 */
}

uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (!monitor_initialized || id != TASK_MONITOR_INS || id >= TASK_MONITOR_COUNT ||
        operation >= TASK_MONITOR_LONG_OPERATION_COUNT || TaskMonitorLongOperationLimitMs(operation) == 0u ||
        long_operation_timeout_latched)
        return 0u;

    uint32_t now = HAL_GetTick();
    TaskMonitor_LongOperationWindow_t *window = &long_operation_windows[id][operation];
    if (window->active)
        return 0u;

    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        uint8_t active = TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);
        uint8_t startup_grace = (operation == TASK_MONITOR_LONG_OPERATION_CALIBRATION &&
                                 monitor_tick_count == 0u && monitor_items[i].alive)
                                    ? 1u
                                    : 0u;
        if (!monitor_items[i].alive ||
            (!active && !startup_grace &&
             (uint32_t)(now - monitor_items[i].last_feed_ms) > monitor_items[i].timeout_ms))
        {
            monitor_items[i].alive = 0u;
            BSP_SafetyLatchOutputsOff();
            RobotSafetySetTaskHealthy(0u);
            if (!monitor_items[i].fault_logged)
            {
                LOGERROR("[monitor] long operation refused: task %s is unhealthy", monitor_items[i].name);
                monitor_items[i].fault_logged = 1u;
            }
            return 0u;
        }
    }

    if (long_operation_timeout_latched)
        return 0u;

    window->started_ms = now;
    window->active = 1u;
    return 1u;
}

uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (id != TASK_MONITOR_INS || id >= TASK_MONITOR_COUNT || operation >= TASK_MONITOR_LONG_OPERATION_COUNT)
        return 0u;

    if (TaskMonitorLongOperationLimitMs(operation) == 0u)
        return 0u;

    TaskMonitor_LongOperationWindow_t *window = &long_operation_windows[id][operation];
    if (!window->active)
        return 0u;

    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    uint8_t within_limit = (uint32_t)(now - window->started_ms) < TaskMonitorLongOperationLimitMs(operation);
    if (!within_limit)
        long_operation_timeout_latched = 1u;

    /* Also expire an enclosing calibration window if Flash save consumed the
       remainder of its overall 15-second budget. This state transition is
       atomic with respect to the daemon monitor tick. */
    for (uint32_t i = 0u; i < TASK_MONITOR_COUNT; ++i)
        (void)TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);

    if (within_limit && !long_operation_timeout_latched)
    {
        uint32_t first_task = (operation == TASK_MONITOR_LONG_OPERATION_FLASH) ? 0u : (uint32_t)id;
        uint32_t end_task = (operation == TASK_MONITOR_LONG_OPERATION_FLASH)
                                ? TASK_MONITOR_COUNT
                                : ((uint32_t)id + 1u);
        for (uint32_t i = first_task; i < end_task; ++i)
        {
            monitor_items[i].last_feed_ms = now;
            monitor_items[i].alive = 1u;
            monitor_items[i].fault_logged = 0u;
        }
    }
    window->active = 0u;
    uint8_t deadline_latched = long_operation_timeout_latched;
    __set_PRIMASK(previous_primask);

    uint8_t all_tasks_healthy = (deadline_latched == 0u) ? 1u : 0u;
    for (uint32_t i = 0u; i < TASK_MONITOR_COUNT; ++i)
    {
        if (operation == TASK_MONITOR_LONG_OPERATION_FLASH && within_limit && !deadline_latched)
            continue; /* Begin verified every task; the CPU stall itself prevents task execution. */
        uint8_t active = TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);
        if (!monitor_items[i].alive ||
            (!active && (uint32_t)(now - monitor_items[i].last_feed_ms) > monitor_items[i].timeout_ms))
            all_tasks_healthy = 0u;
    }

    if (!within_limit || !all_tasks_healthy)
    {
        TaskMonitorItem_t *item = &monitor_items[id];
        if (!within_limit)
            item->alive = 0u;
        if ((!within_limit || !all_tasks_healthy) && !item->fault_logged)
        {
            BSP_SafetyLatchOutputsOff();
            if (!within_limit || deadline_latched)
                LOGERROR("[monitor] bounded long operation exceeded its deadline or another monitor fault occurred");
            else
                LOGERROR("[monitor] another task faulted during bounded long operation");
            item->fault_logged = 1u;
        }
        RobotSafetySetTaskHealthy(0u);
        return 0u;
    }

    monitor_items[id].last_feed_ms = now;
    monitor_items[id].alive = 1u;
    return 1u;
}

uint8_t TaskMonitorAllAlive(void)
{
    if (long_operation_timeout_latched)
        return 0u;
    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        if (!monitor_items[i].alive)
            return 0;
    }
    return 1;
}

void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status)
{
    if (id >= TASK_MONITOR_COUNT || status == NULL)
        return;

    const TaskMonitorItem_t *item = &monitor_items[id];
    status->name = item->name;
    status->alive = item->alive;
    status->fault = (uint8_t)(item->alive == 0);
    status->timeout_ms = item->timeout_ms;
}
