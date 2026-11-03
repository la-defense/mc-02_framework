#include "platform_task_stats.h"
#include "FreeRTOS.h"
#include "task.h"
#include "test_check.h"
#include <string.h>

static uint8_t task_count_read;

UBaseType_t uxTaskGetNumberOfTasks(void)
{
    task_count_read++; // 用两种返回数量模拟捕获前后同时创建任务。
    return task_count_read == 1u ? 2u : 4u;
}

UBaseType_t uxTaskGetSystemState(TaskStatus_t *records,
                                 UBaseType_t capacity,
                                 uint32_t *total_runtime)
{
    if (capacity < 3u)
        return 0u;

    records[0] = (TaskStatus_t){"LCD-task-name-is-long", 100u, 2u, 64u}; // 检查长名称被限定复制。
    records[1] = (TaskStatus_t){NULL, 200u, 1u, 32u}; // 检查空任务名仍会得到结束符。
    records[2] = (TaskStatus_t){"idle", 700u, 0u, 128u}; // 提供另一条正常短名称记录。
    *total_runtime = 1000u; // 模拟 FreeRTOS 汇总的系统运行计数。
    return 3u;
}

int main(void)
{
    PlatformTaskStatsInfo_t info = {0}; // 捕获结果只包含框架自有的固定宽度字段。
    TEST_CHECK(PlatformTaskStatsCapture(&info) == 1u);
    TEST_CHECK(info.captured_count == 3u);
    TEST_CHECK(info.total_task_count == 4u);
    TEST_CHECK(info.total_runtime_count == 1000u);

    PlatformTaskStatsEntry_t entry;
    memset(&entry, 0xA5, sizeof(entry));
    TEST_CHECK(PlatformTaskStatsGetEntry(0u, &entry) == 1u);
    TEST_CHECK(strcmp(entry.name, "LCD-task-na") == 0);
    TEST_CHECK(entry.runtime_counter == 100u);
    TEST_CHECK(entry.priority == 2u);
    TEST_CHECK(entry.stack_high_water_words == 64u);

    TEST_CHECK(PlatformTaskStatsGetEntry(1u, &entry) == 1u);
    TEST_CHECK(entry.name[0] == '\0');
    TEST_CHECK(entry.runtime_counter == 200u);

    PlatformTaskStatsEntry_t old_entry;
    memset(&old_entry, 0x5A, sizeof(old_entry));
    entry = old_entry;
    TEST_CHECK(PlatformTaskStatsGetEntry(info.captured_count, &entry) == 0u);
    TEST_CHECK(memcmp(&entry, &old_entry, sizeof(entry)) == 0);
    TEST_CHECK(PlatformTaskStatsCapture(NULL) == 0u);
    return 0;
}
