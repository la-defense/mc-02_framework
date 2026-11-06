/** @file platform_task_batch_test.c
 * @brief 用调度器替身运行生产批次入口，检查调用上下文及暂停配对。
 */
#include "platform_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "test_check.h"

static BaseType_t scheduler_state = taskSCHEDULER_NOT_STARTED;
static uint8_t in_interrupt;
static unsigned suspends, resumes;
uint8_t PlatformIrqIsInHandler(void) { return in_interrupt; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler_state; }
void vTaskSuspendAll(void)
{
    TEST_CHECK(scheduler_state == taskSCHEDULER_RUNNING); // 未运行时不能暂停。
    scheduler_state = taskSCHEDULER_SUSPENDED; // 记录当前任务独占批次。
    ++suspends; // 检查重复开始不会增加暂停层数。
}
BaseType_t xTaskResumeAll(void)
{
    TEST_CHECK(scheduler_state == taskSCHEDULER_SUSPENDED); // 没有暂停时不得恢复。
    scheduler_state = taskSCHEDULER_RUNNING; // 模拟恢复调度。
    ++resumes; // 记录配对次数。
    return 0; // 未触发立即切换仍代表恢复成功。
}
int main(void)
{
    TEST_CHECK(!PlatformTaskBatchBegin() && !PlatformTaskBatchEnd()); // 调度器未运行时拒绝。
    scheduler_state = taskSCHEDULER_SUSPENDED; // 其他代码已暂停调度器时不能嵌套。
    TEST_CHECK(!PlatformTaskBatchBegin());
    scheduler_state = taskSCHEDULER_RUNNING; // 进入正常任务运行状态。
    in_interrupt = 1u; // 中断不能申请或借用批次。
    TEST_CHECK(!PlatformTaskBatchBegin() && !PlatformTaskBatchIsActive());
    in_interrupt = 0u;
    TEST_CHECK(PlatformTaskBatchBegin() && PlatformTaskBatchIsActive()); // 协调任务可以开始。
    TEST_CHECK(!PlatformTaskBatchBegin() && suspends == 1u); // 重复开始不能多暂停一次。
    in_interrupt = 1u;
    TEST_CHECK(!PlatformTaskBatchIsActive() && !PlatformTaskBatchEnd()); // 中断不能结束任务持有的批次。
    in_interrupt = 0u;
    TEST_CHECK(PlatformTaskBatchEnd() && !PlatformTaskBatchIsActive()); // 恢复后撤销创建许可。
    TEST_CHECK(!PlatformTaskBatchEnd() && resumes == 1u); // 重复结束不能再次恢复。
    return 0;
}
