/** @file task_monitor.h
 * @brief 参数维护测试使用的任务监控接口。
 */
#include <stdint.h>
typedef enum { TASK_MONITOR_INS = 0 } TaskMonitor_Id_e;
typedef enum { TASK_MONITOR_LONG_OPERATION_FLASH = 1 } TaskMonitor_LongOperation_e;
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
