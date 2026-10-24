/** @file daemon.c
* @brief 保存模块计数，周期递减并调用一次离线处理，收到新数据后可重新开始下一轮。
*/
#include "daemon.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_dwt.h" // 后续通过定时器来计时?
#include "stdlib.h"
#include "memory.h"
#include "buzzer.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (config == NULL)
        return NULL;

    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count;
    instance->callback = config->callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count;

    // 对象完整初始化后才发布，避免守护任务读到半成品。
    taskENTER_CRITICAL();
    if (idx >= DAEMON_MX_CNT)
    {
        taskEXIT_CRITICAL();
        free(instance);
        return NULL;
    }
    daemon_instances[idx++] = instance;
    taskEXIT_CRITICAL();
    return instance;
}

uint8_t DaemonUnregister(DaemonInstance *instance)
{
    if (instance == NULL)
        return 0u;

    uint8_t removed = 0u;
    taskENTER_CRITICAL();
    for (uint8_t i = 0u; i < idx; ++i)
    {
        if (daemon_instances[i] != instance)
            continue;
        for (uint8_t j = (uint8_t)(i + 1u); j < idx; ++j)
            daemon_instances[j - 1u] = daemon_instances[j];
        daemon_instances[--idx] = NULL;
        removed = 1u;
        break;
    }
    taskEXIT_CRITICAL();

    if (removed != 0u)
        free(instance);
    return removed;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    if (instance == NULL)
        return;
    taskENTER_CRITICAL();
    instance->temp_count = instance->reload_count;
    instance->offline_flag = 0;
    taskEXIT_CRITICAL();
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    if (instance == NULL)
        return 0u;
    taskENTER_CRITICAL();
    uint8_t online = (uint8_t)(instance->temp_count > 0u);
    taskEXIT_CRITICAL();
    return online;
}

void DaemonTask()
{
    taskENTER_CRITICAL();
    uint8_t instance_count = idx;
    taskEXIT_CRITICAL();

    for (uint8_t i = 0u; i < instance_count; ++i)
    {
        offline_callback callback = NULL;
        void *owner_id = NULL;
        taskENTER_CRITICAL();
        if (i < idx)
        {
            DaemonInstance *instance = daemon_instances[i];
            if (instance != NULL)
            {
                if (instance->temp_count > 0u)
                    instance->temp_count--;
                else if (instance->offline_flag == 0u && instance->callback != NULL)
                {
                    // 先记录已报告状态，再在临界区外通知，避免覆盖回调中的重新喂狗。
                    instance->offline_flag = 1u;
                    callback = instance->callback;
                    owner_id = instance->owner_id;
                }
            }
        }
        taskEXIT_CRITICAL();

        if (callback != NULL)
        {
            callback(owner_id);
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}
// owner_id 让同一个回调能够区分多个模块对象。
