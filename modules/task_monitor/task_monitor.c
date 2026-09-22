#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "robot_safety.h"
#include "bsp_log.h"
#include "main.h"

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

static uint8_t monitor_paused = 0;

void TaskMonitorInit(void)
{
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        monitor_items[i].last_feed_ms = now;
        monitor_items[i].alive = 1;
        monitor_items[i].fault_logged = 0;
    }
    monitor_paused = 0;   /* 正式行为: 关键任务都健康时才喂狗 */
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
    if (monitor_paused)
    {
        BSP_WatchdogFeed();
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t all_alive = 1;

    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        TaskMonitorItem_t *item = &monitor_items[i];
        if ((uint32_t)(now - item->last_feed_ms) > item->timeout_ms)
        {
            item->alive = 0;
            all_alive = 0;
            if (!item->fault_logged)
            {
                LOGERROR("[monitor] task %s timeout (%ums)", item->name, (unsigned)item->timeout_ms);
                item->fault_logged = 1;
            }
        }
        else
        {
            item->alive = 1;
            item->fault_logged = 0;
        }
    }

    RobotSafetySetTaskHealthy(all_alive);

    if (all_alive)
        BSP_WatchdogFeed();
    /* 任一关键任务超时: 不喂狗, 由 IWDG 复位 */
}

void TaskMonitorPause(void)
{
    monitor_paused = 1;
    BSP_WatchdogFeed();
}

void TaskMonitorResume(void)
{
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        monitor_items[i].last_feed_ms = now;
        monitor_items[i].alive = 1;
        monitor_items[i].fault_logged = 0;
    }
    monitor_paused = 0;
}

uint8_t TaskMonitorAllAlive(void)
{
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
