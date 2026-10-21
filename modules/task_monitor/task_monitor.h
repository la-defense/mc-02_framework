/** @file task_monitor.h
* @brief 监视 INS、MOTOR、ROBOT、DAEMON及UART_TX运行间隔，全部正常时才喂看门狗。
* @note 时间均为HAL毫秒；常规允许间隔分别为5、5、20、30、30ms，等于时限仍允许。
*/
#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include <stdint.h>

typedef enum
{
    TASK_MONITOR_INS = 0,
    TASK_MONITOR_MOTOR,
    TASK_MONITOR_ROBOT,
    TASK_MONITOR_DAEMON,
    TASK_MONITOR_UART_TX,
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

/** @brief 清空故障和长操作记录，为全部任务设置当前起始时间；启动监控前调用一次。 */
void TaskMonitorInit(void);
/** @brief 记录指定任务刚执行过。
* @param[in] id 有效任务编号；超出上限时不处理，不会替其他任务更新计时。
*/
void TaskMonitorFeed(TaskMonitor_Id_e id);
/** @brief 调度器运行时每等 1 ms 更新调用方任务的运行记录；未运行时用 DWT 等待。
* @param[in] id 调用方任务编号，须来自 TaskMonitor_Id_e。
* @param[in] milliseconds 等待时长，单位为 ms；为 0 不处理。
* @note 不代替其他任务报告正常；DWT 等待路径不更新运行记录。
*/
void TaskMonitorDelayMs(TaskMonitor_Id_e id, uint32_t milliseconds);
/** @brief 周期检查各任务，报告故障并决定是否喂狗；由 Daemon 约每 10 ms 调用。 */
void TaskMonitorTick(void);
/** @brief 为 INS 申请有时限的长操作时间，开始前检查所有任务是否正常。
* @param[in] id 目前只允许 TASK_MONITOR_INS。
* @param[in] operation 标定最多 15000 ms，Flash 保存最多 8000 ms；达到时限即失败。
* @return 1 为开始成功，0 为未初始化、已有故障、重复申请或条件不符。
* @note 标定只延长 INS 检查，Flash 操作期间允许所有任务暂停；期间观察到的超时保留至重新初始化。
*/
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
/** @brief 结束对应长操作，检查总耗时及其他任务。
* @param[in] id 须为先前申请的 INS。
* @param[in] operation 须为已开始且尚未结束的操作类型。
* @return 1 为按时结束且任务正常；0 为申请不匹配、到时或发生故障，故障不会因结束而清除。
* @note 按时结束标定只更新 INS 计时，按时结束 Flash 更新全部任务计时；标定内保存 Flash 仍共用标定总时限。
*/
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation);
/** @brief 读取已保存的任务正常标记，不在此处重新检查当前时间。
* @return 1 为全部标记正常且没有保留的长操作故障，否则为 0；须配合周期 Tick 使用。
*/
uint8_t TaskMonitorAllAlive(void);
/** @brief 读取指定任务的最近监控结果。
* @param[in] id 有效任务编号。
* @param[out] status 输出名称、正常标记及允许间隔；NULL 或编号超出上限时保持输出不变。
*/
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *status);

#endif // !TASK_MONITOR_H
