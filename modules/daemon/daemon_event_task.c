/** @file daemon_event_task.c
 * @brief 为选择独立投递方式的来源创建一个固定事件处理任务。
 * @note 仅在所有来源注册完成、调度器启动前调用；失败保留来源状态供诊断。
 */
#include "daemon.h"

#include "cmsis_os.h"

#include <stddef.h>

#define DAEMON_EVENT_TASK_STACK_WORDS 512u
#define DAEMON_EVENT_TASK_BATCH DAEMON_EVENT_MAX_DEPTH

static void DaemonIndependentEventTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        uint8_t processed = 0u;
        (void)DaemonProcessTaskEvents(DAEMON_EVENT_TASK_CONSUMER,
                                      DAEMON_EVENT_TASK_BATCH, &processed);
        osDelay(1u);
    }
}

DaemonStatus_t DaemonEventTaskInit(void)
{
    if (DaemonHasIndependentSources() == 0u)
    {
        DaemonSetEventTaskResult(0u, 0u);
        return DAEMON_NO_TASK_REQUIRED;
    }

    DaemonStats_t stats = {0};
    if (DaemonGetStats(NULL, &stats) == DAEMON_OK && stats.task_started != 0u)
        return DAEMON_OK;

    osThreadDef(daemon_event_task, DaemonIndependentEventTask, osPriorityBelowNormal, 0,
                DAEMON_EVENT_TASK_STACK_WORDS);
    // 任务可能在 osThreadCreate 返回前运行，先开放它的消费者编号。
    DaemonSetEventTaskResult(1u, 0u);
    if (osThreadCreate(osThread(daemon_event_task), NULL) == NULL)
    {
        DaemonSetEventTaskResult(0u, 1u);
        return DAEMON_ERR_TASK_CREATE;
    }
    return DAEMON_OK;
}
