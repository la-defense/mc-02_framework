/** @file platform_task_stats_freertos.c
 * @brief 把 FreeRTOS 任务统计转换为框架自有的固定长度记录。
 */
#include "platform_task_stats.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#if (configUSE_TRACE_FACILITY == 1)
static TaskStatus_t freertos_task_records[PLATFORM_TASK_STATS_MAX_COUNT];
static UBaseType_t freertos_captured_count;
#endif

uint8_t PlatformTaskStatsCapture(PlatformTaskStatsInfo_t *info)
{
    if (info == NULL)
        return 0u;

#if (configUSE_TRACE_FACILITY == 1)
    const UBaseType_t count_before = uxTaskGetNumberOfTasks(); // 记录读取前的任务总数，观察并发创建或删除。
    uint32_t total_runtime = 0u;
    const UBaseType_t captured = uxTaskGetSystemState(
        freertos_task_records, PLATFORM_TASK_STATS_MAX_COUNT, &total_runtime); // 只写入固定容量的任务记录。
    const UBaseType_t count_after = uxTaskGetNumberOfTasks(); // 用读取后的任务数补足并发变化。
    PlatformTaskStatsInfo_t result;

    // FreeRTOS 可能在读取期间增删任务，数量用读取前后与实际记录的最大值表示。
    result.total_task_count = (uint32_t)count_before;
    if ((uint32_t)count_after > result.total_task_count)
        result.total_task_count = (uint32_t)count_after;
    if ((uint32_t)captured > result.total_task_count)
        result.total_task_count = (uint32_t)captured;
    result.captured_count = (uint32_t)captured; // 记录实际写入的记录数量。
    result.total_runtime_count = total_runtime; // 保留后端给出的系统累计运行计数。

    freertos_captured_count = captured; // 允许后续按序读取本次静态记录。
    *info = result; // 所有读取完成后再一次性交付计数信息。
    return 1u;
#else
    return 0u;
#endif
}

uint8_t PlatformTaskStatsGetEntry(uint32_t index, PlatformTaskStatsEntry_t *entry)
{
    if (entry == NULL)
        return 0u;

#if (configUSE_TRACE_FACILITY == 1)
    if (index >= (uint32_t)freertos_captured_count)
        return 0u;

    const TaskStatus_t *source = &freertos_task_records[index]; // 只在适配层读取 RTOS 原生记录。
    PlatformTaskStatsEntry_t result;
    memset(&result, 0, sizeof(result)); // 先清空结果，确保没有名称时也有结束符。
    if (source->pcTaskName != NULL)
    {
        for (uint32_t name_index = 0u;
             name_index < PLATFORM_TASK_STATS_NAME_SIZE - 1u &&
                 source->pcTaskName[name_index] != '\0';
             ++name_index)
            result.name[name_index] = source->pcTaskName[name_index]; // 最多复制11个字符。
    }

    result.runtime_counter = source->ulRunTimeCounter; // 保留 FreeRTOS 当前累计计数。
    result.priority = (uint32_t)source->uxCurrentPriority; // 转为框架固定宽度整数。
    result.stack_high_water_words = (uint32_t)source->usStackHighWaterMark; // 栈余量继续按字数报告。
    *entry = result; // 记录完整后再覆盖调用者输出。
    return 1u;
#else
    (void)index;
    return 0u;
#endif
}
