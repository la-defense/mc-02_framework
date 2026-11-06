/** @file platform_task_batch_freertos.c
 * @brief 使用 FreeRTOS 调度器暂停保护短暂的启动任务创建批次。
 */
#include "platform_os.h"
#include "platform_irq.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint8_t task_batch_active; // 调度器暂停期间只有协调任务能修改此标记。

uint8_t PlatformTaskBatchBegin(void)
{
    if (PlatformIrqIsInHandler() != 0u) return 0u; // 中断中不能暂停任务调度。
    if (task_batch_active != 0u) return 0u; // 重复开始不能增加未配对的暂停层数。
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return 0u; // 只接受正常运行且未被其他调用暂停的调度器。
    vTaskSuspendAll(); // 仅暂停任务切换，硬件中断仍可正常运行。
    task_batch_active = 1u; // 完成暂停后才允许启动适配器创建任务。
    return 1u;
}

uint8_t PlatformTaskBatchIsActive(void)
{
    return (uint8_t)(PlatformIrqIsInHandler() == 0u && task_batch_active != 0u); // 中断不得借用任务创建许可。
}

uint8_t PlatformTaskBatchEnd(void)
{
    if (PlatformIrqIsInHandler() != 0u || task_batch_active == 0u) return 0u; // 非法结束不能改变调度器。
    task_batch_active = 0u; // 先撤销创建许可，避免恢复后其他任务沿用。
    (void)xTaskResumeAll(); // 恢复本批次暂停；返回值只表示是否已触发任务切换。
    return 1u;
}
