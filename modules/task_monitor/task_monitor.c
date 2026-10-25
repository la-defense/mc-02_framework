/** @file task_monitor.c
* @brief 记录各关键任务最近执行时间，处理标定与 Flash 等长操作，并决定是否喂 IWDG。
*/
#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "robot_safety.h"
#include "bsp_log.h"
#include "main.h"
#include "bsp_safety.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
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
    [TASK_MONITOR_UART_TX] = {.name = "UART_TX", .timeout_ms = 30},
};

static uint8_t monitor_initialized = 0u;
static uint8_t long_operation_timeout_latched = 0u;
static uint8_t long_operation_fault_latched = 0u;
static uint32_t monitor_tick_count = 0u;

typedef struct
{
    uint32_t started_ms;
    uint8_t active;
} TaskMonitor_LongOperationWindow_t;

static TaskMonitor_LongOperationWindow_t long_operation_windows[TASK_MONITOR_COUNT][TASK_MONITOR_LONG_OPERATION_COUNT];

// 返回该类长操作允许的毫秒数；未知类型返回 0。
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

// 检查 INS 的各项长操作；达到时限后保留失败记录，不再当作允许等待。
static uint8_t TaskMonitorHasActiveLongOperation(TaskMonitor_Id_e id, uint32_t now)
{
    uint8_t active = 0u;
    for (uint32_t operation = 0u; operation < TASK_MONITOR_LONG_OPERATION_COUNT; ++operation)
    {
        // Flash 擦写会让整个内核暂停，因此暂缓全部任务计时；标定只暂缓 INS。
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

// 找到任何未结束的长操作就返回，用于保留其间发生的任务故障。
static uint8_t TaskMonitorHasAnyActiveLongOperation(void)
{
    for (uint32_t id = 0u; id < TASK_MONITOR_COUNT; ++id)
    {
        for (uint32_t operation = 0u; operation < TASK_MONITOR_LONG_OPERATION_COUNT; ++operation)
        {
            if (long_operation_windows[id][operation].active)
                return 1u;
        }
    }
    return 0u;
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
    long_operation_fault_latched = 0u;
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

void TaskMonitorDelayMs(TaskMonitor_Id_e id, uint32_t milliseconds)
{
    if (id >= TASK_MONITOR_COUNT || milliseconds == 0u)
        return;

    if (!osKernelRunning())
    {
        DWT_Delay((float)milliseconds / 1000.0f);
        return;
    }

    // 分成 1 ms 等待，让其他任务运行，同时只更新当前任务的执行记录。
    while (milliseconds > 0u)
    {
        osDelay(1u);
        TaskMonitorFeed(id);
        milliseconds--;
    }
}

void TaskMonitorTick(void)
{
    monitor_tick_count++;
    uint32_t now = HAL_GetTick();
    uint8_t all_alive = (long_operation_timeout_latched == 0u && long_operation_fault_latched == 0u)
                            ? 1u
                            : 0u;

    // 逐项检查是否超过各自间隔；首次超时先关闭输出，再报告故障。
    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        TaskMonitorItem_t *item = &monitor_items[i];
        uint8_t long_operation_active = TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);
        if (!long_operation_active && (uint32_t)(now - item->last_feed_ms) > item->timeout_ms)
        {
            item->alive = 0;
            all_alive = 0;
            if (TaskMonitorHasAnyActiveLongOperation())
                long_operation_fault_latched = 1u;
            if (!item->fault_logged)
            {
                BSP_SafetyLatchOutputsOff();
                BSP_LOG_EVENT_2(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_01, BSP_LOG_FIELD_CSTR_LIMIT((item->name), 48u), BSP_LOG_FIELD_U32((unsigned)item->timeout_ms));
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

    if (long_operation_timeout_latched || long_operation_fault_latched)
        all_alive = 0u;

    RobotSafetySetTaskHealthy(all_alive);

    // 所有任务都正常才喂狗；保留的长操作故障也会持续阻止喂狗。
    if (all_alive)
        BSP_WatchdogFeed();
    /* 任一关键任务超时: 不喂狗, 由 IWDG 复位 */
}

uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (!monitor_initialized || id != TASK_MONITOR_INS || id >= TASK_MONITOR_COUNT ||
        operation >= TASK_MONITOR_LONG_OPERATION_COUNT || TaskMonitorLongOperationLimitMs(operation) == 0u ||
        long_operation_timeout_latched || long_operation_fault_latched)
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
                BSP_LOG_EVENT_1(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_02, BSP_LOG_FIELD_CSTR_LIMIT((monitor_items[i].name), 48u));
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

    // 同时检查外层标定是否耗尽 15 秒；关中断期间完成，避免 Daemon 在中途读取。
    for (uint32_t i = 0u; i < TASK_MONITOR_COUNT; ++i)
        (void)TaskMonitorHasActiveLongOperation((TaskMonitor_Id_e)i, now);

    uint8_t operation_fault_latched = (uint8_t)(long_operation_timeout_latched || long_operation_fault_latched);
    // 仅按时且期间没有故障时更新计时；超时记录不能靠结束操作清除。
    if (within_limit && !operation_fault_latched)
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
    operation_fault_latched = (uint8_t)(long_operation_timeout_latched || long_operation_fault_latched);
    __set_PRIMASK(previous_primask);

    uint8_t all_tasks_healthy = (operation_fault_latched == 0u) ? 1u : 0u;
    for (uint32_t i = 0u; i < TASK_MONITOR_COUNT; ++i)
    {
        if (operation == TASK_MONITOR_LONG_OPERATION_FLASH && within_limit && !operation_fault_latched)
            continue; // 开始前已检查各任务；这段 Flash 暂停期间没有任务能执行，按时结束后重新计时。
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
            if (!within_limit || operation_fault_latched)
                BSP_LOG_EVENT_0(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_03);
            else
                BSP_LOG_EVENT_0(EV_MODULES_TASK_MONITOR_TASK_MONITOR_C_04);
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
    if (long_operation_timeout_latched || long_operation_fault_latched)
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

    const uint32_t previous = __get_PRIMASK();
    __disable_irq();
    const TaskMonitorItem_t *item = &monitor_items[id];
    status->name = item->name;
    status->alive = item->alive;
    status->fault = (uint8_t)(item->alive == 0);
    status->timeout_ms = item->timeout_ms;
    __set_PRIMASK(previous);
}
