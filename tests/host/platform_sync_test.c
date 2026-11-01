/** @file platform_sync_test.c
 * @brief 验证框架任务临界区的嵌套和现有毫秒时钟回绕。
 */
#include "platform_sync.h"
#include "test_check.h"
#include <stdint.h>

static uint32_t tick_ms; // 测试自行控制后端时钟，覆盖回绕前后。
uint32_t HostGetPrimask(void); // 由测试的可嵌套互斥模型提供当前线程深度。

uint32_t HAL_GetTick(void)
{
    return tick_ms; // 只替换时钟来源，平台适配器使用生产实现。
}

int main(void)
{
    TEST_CHECK(HostGetPrimask() == 0u); // 初始时没有进入临界区。
    PlatformTaskCriticalEnter(); // 外层进入后应保持一次保护。
    PlatformTaskCriticalEnter(); // 内层进入不能取消外层保护。
    TEST_CHECK(HostGetPrimask() == 2u);
    PlatformTaskCriticalExit(); // 内层退出后外层仍然有效。
    TEST_CHECK(HostGetPrimask() == 1u);
    PlatformTaskCriticalExit(); // 两次配对退出后释放保护。
    TEST_CHECK(HostGetPrimask() == 0u);
    tick_ms = UINT32_MAX; // 在回绕前记录现有计数。
    const uint32_t before = PlatformTimeNowMs();
    tick_ms = 0u; // 模拟后端计数回绕到零。
    TEST_CHECK((uint32_t)(PlatformTimeNowMs() - before) == 1u); // 无符号差值仍是 1 ms。
    return 0;
}
