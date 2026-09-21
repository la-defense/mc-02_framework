#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>

typedef enum
{
    TASK_MONITOR_INS = 0,
    TASK_MONITOR_MOTOR,
    TASK_MONITOR_ROBOT,
    TASK_MONITOR_DAEMON,
    TASK_MONITOR_COUNT
} TaskMonitor_Id_e;

typedef struct
{
    const char *name;
    uint8_t alive;
    uint8_t fault;
    uint32_t timeout_ms;
} TaskMonitorStatus_t;

void TaskMonitorInit(void);
void TaskMonitorFeed(TaskMonitor_Id_e id);
void TaskMonitorTick(void);
void TaskMonitorPause(void);
void TaskMonitorResume(void);
uint8_t TaskMonitorAllAlive(void);
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status);

#endif // !TASK_MONITOR_H
