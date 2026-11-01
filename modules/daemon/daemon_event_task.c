/** @file daemon_event_task.c
 * @brief 为选择独立投递方式的来源创建一个固定事件处理任务。
 * @note 仅在所有来源注册完成、调度器启动前调用；失败保留来源状态供诊断。
 */
#include "daemon.h"

#include "platform_os.h"

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
        PlatformTaskDelayMs(1u); // 给其他任务运行机会，保持原来的毫秒等待。
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

    const PlatformTaskConfig_t task = {
        .name = "daemon_event_task", // 与原 CMSIS 任务名称一致。
        .entry = DaemonIndependentEventTask, // 在任务中派发来源事件。
        .argument = NULL, // 当前任务不需要外部参数。
        .stack_words = DAEMON_EVENT_TASK_STACK_WORDS, // 保持原有栈字数。
        .priority = PLATFORM_TASK_BELOW_NORMAL, // 保持原有后台优先级。
    };
    // 任务可能在 osThreadCreate 返回前运行，先开放它的消费者编号。
    DaemonSetEventTaskResult(1u, 0u);
    if (PlatformTaskCreate(&task) == 0u)
    {
        DaemonSetEventTaskResult(0u, 1u);
        return DAEMON_ERR_TASK_CREATE;
    }
    return DAEMON_OK;
}
