/** @file daemon.c
* @brief 保存模块计数，周期递减并调用一次离线处理，收到新数据后可重新开始下一轮。
*/
#include "daemon.h"
#include "bsp_dwt.h" // 后续通过定时器来计时?
#include "stdlib.h"
#include "memory.h"
#include "buzzer.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    if (config == NULL || idx >= DAEMON_MX_CNT)
        return NULL;

    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数

    // 配置全部准备好才加入遍历表，避免任务读到半初始化对象。
    daemon_instances[idx] = instance;
    idx++;
    return instance;
}

uint8_t DaemonUnregister(DaemonInstance *instance)
{
    if (instance == NULL)
        return 0u;
    for (uint8_t i = 0u; i < idx; ++i)
    {
        if (daemon_instances[i] != instance)
            continue;
        for (uint8_t j = (uint8_t)(i + 1u); j < idx; ++j)
            daemon_instances[j - 1u] = daemon_instances[j];
        daemon_instances[--idx] = NULL;
        free(instance);
        return 1u;
    }
    return 0u;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    if (instance == NULL)
        return;
    instance->temp_count = instance->reload_count;
    instance->offline_flag = 0; // 重新上线, 允许下次离线时再次告警
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return (uint8_t)(instance != NULL && instance->temp_count > 0u);
}

void DaemonTask()
{
    DaemonInstance *dins; // 提高可读性同时降低访存开销
    for (size_t i = 0; i < idx; ++i)
    {

        dins = daemon_instances[i];
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            dins->temp_count--;
        else if (!dins->offline_flag && dins->callback) // 本次进入时已为 0，且还没报告过离线，才调用一次回调。
        {
            dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
            dins->offline_flag = 1;          // 本次离线已告警, 避免每个周期重复触发
            // @todo 为蜂鸣器/led等增加离线报警的功能,非常关键!
        }
    }
}
// 原文件曾引用 Copilot 说明 owner_id：它用于让同一回调区分多个模块对象。
