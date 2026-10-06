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

typedef enum
{
    TASK_MONITOR_LONG_OPERATION_CALIBRATION = 0,
    TASK_MONITOR_LONG_OPERATION_FLASH,
    TASK_MONITOR_LONG_OPERATION_COUNT
} TaskMonitor_LongOperation_e;

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
/* Only INS may use these bounded windows. Other tasks remain monitored.
   A task fault observed during an active window remains latched until reset. */
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
uint8_t TaskMonitorAllAlive(void);
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status);

#endif // !TASK_MONITOR_H
