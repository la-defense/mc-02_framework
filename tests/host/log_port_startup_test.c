/** @file log_port_startup_test.c
 * @brief 直接测试生产日志端口的启动上下文、失败重试及句柄发布。
 */
#include "bsp_log_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "test_check.h"
#include <setjmp.h>
#include <string.h>

SCB_Type log_test_scb; // 提供编译所需的片上寄存器替身。
uint32_t log_test_ipsr; // 非零模拟中断上下文。
static uint8_t scheduler_running, batch_active, fail_create; // 控制创建上下文和失败注入。
static unsigned create_count, callback_count; // 记录实际创建和任务入口调用。
static void (*task_entry)(void *); // 捕获生产端口交给 FreeRTOS 的入口。
static TaskHandle_t current_task; // 模拟运行中的任务句柄。
static TaskHandle_t created_task; // 保存创建成功后返回的控制块地址。
static jmp_buf task_finished; // 从常驻任务入口返回主机测试。

int32_t osKernelRunning(void) { return scheduler_running; }
uint8_t PlatformTaskBatchIsActive(void) { return batch_active; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current_task; }
void vTaskSuspend(TaskHandle_t handle) { (void)handle; }

TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name, uint32_t depth,
                               void *argument, uint32_t priority, StackType_t *stack,
                               StaticTask_t *control)
{
    TEST_CHECK(entry != NULL && strcmp(name, "logtask") == 0); // 核对实际创建的任务入口。
    TEST_CHECK(depth == 1024u && argument == NULL && priority == 2u); // 保留原有资源配置。
    TEST_CHECK(stack != NULL && control != NULL); // 静态栈和控制块必须存在。
    ++create_count; // 统计真正到达任务创建后端的次数。
    if (fail_create) return NULL; // 模拟资源不足，不发布句柄。
    task_entry = entry; // 保存入口，稍后模拟调度器运行任务。
    created_task = control; // 与生产端口保存的句柄保持一致。
    return control; // 使用测试控制块地址作为稳定句柄。
}

static void worker(void const *argument)
{
    TEST_CHECK(argument == NULL); // 生产任务应保留原有空参数。
    TEST_CHECK(BSPLogPortIsWorker()); // 任务运行前句柄必须已发布。
    ++callback_count; // 确认创建时设置的业务入口被调用。
    longjmp(task_finished, 1); // 跳出生产常驻循环，不进入真实调度。
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 各场景分别运行，避免静态任务句柄跨场景残留。
    TEST_CHECK(!BSPLogPortStartWorker(NULL)); // 空入口不能创建任务。
    if (strcmp(argv[1], "before") == 0)
    {
        TEST_CHECK(BSPLogPortStartWorker(worker) && create_count == 1u); // 调度器启动前仍允许创建。
        current_task = created_task; // 模拟调度器开始运行这个任务。
        if (setjmp(task_finished) == 0) task_entry(NULL); // 检查任务入口与句柄已发布。
        TEST_CHECK(callback_count == 1u);
        return 0;
    }
    TEST_CHECK(strcmp(argv[1], "batch") == 0); // 未声明的场景不能静默通过。
    scheduler_running = 1u; // 模拟调度器已经启动。
    TEST_CHECK(!BSPLogPortStartWorker(worker) && create_count == 0u); // 批次外拒绝运行时创建。
    batch_active = 1u; // 启动协调者已暂停任务切换。
    log_test_ipsr = 1u; // 中断不能借用协调任务持有的批次。
    TEST_CHECK(!BSPLogPortStartWorker(worker) && create_count == 0u);
    log_test_ipsr = 0u;
    fail_create = 1u; // 模拟首次创建失败。
    TEST_CHECK(!BSPLogPortStartWorker(worker) && create_count == 1u);
    fail_create = 0u; // 相同启动批次内允许恢复后重试。
    TEST_CHECK(BSPLogPortStartWorker(worker) && create_count == 2u);
    TEST_CHECK(!BSPLogPortStartWorker(worker) && create_count == 2u); // 成功后禁止重复创建。
    current_task = (TaskHandle_t)1; // 非日志任务不能错误认作日志任务。
    TEST_CHECK(!BSPLogPortIsWorker());
    current_task = created_task; // 模拟调度器在创建完成后运行新任务。
    if (setjmp(task_finished) == 0) task_entry(NULL); // 从生产入口进入已登记的业务函数。
    TEST_CHECK(callback_count == 1u); // 业务入口应且仅应运行一次。
    return 0;
}
