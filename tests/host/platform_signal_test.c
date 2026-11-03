/** @file platform_signal_test.c
 * @brief 验证任务信号平台适配的参数、合并设置与等待结果。
 * @note 用 CMSIS 替身检查调用边界，不模拟真实中断或调度时限。
 */
#include "platform_signal.h"
#include "cmsis_os.h"
#include "test_check.h"

static uint32_t calls_set, calls_wait, recorded_bits, recorded_timeout;
static int32_t set_result;
static osEvent wait_result;

int32_t osSignalSet(osThreadId thread, int32_t signals)
{
    TEST_CHECK(thread == (void *)1);
    ++calls_set;
    recorded_bits = (uint32_t)signals;
    return set_result;
}
osEvent osSignalWait(int32_t signals, uint32_t milliseconds)
{
    ++calls_wait;
    recorded_bits = (uint32_t)signals;
    recorded_timeout = milliseconds;
    return wait_result;
}

int main(void)
{
    uint32_t observed = 0x1234u;
    TEST_CHECK(!PlatformSignalSet(NULL, 1u));
    TEST_CHECK(!PlatformSignalSet((void *)1, 0u));
    TEST_CHECK(!PlatformSignalSet((void *)1, 0x80000000u));
    TEST_CHECK(calls_set == 0u); // 非法输入不能调用 RTOS。
    set_result = 3;
    TEST_CHECK(PlatformSignalSet((void *)1, 5u) && calls_set == 1u && recorded_bits == 5u);
    set_result = (int32_t)0x80000000u;
    TEST_CHECK(!PlatformSignalSet((void *)1, 1u)); // 后端错误不冒充已接受。

    TEST_CHECK(PlatformSignalWait(0u, 0u, &observed) == PLATFORM_SIGNAL_INVALID);
    TEST_CHECK(PlatformSignalWait(1u, 0u, NULL) == PLATFORM_SIGNAL_INVALID);
    TEST_CHECK(calls_wait == 0u && observed == 0x1234u);
    wait_result.status = osEventSignal;
    wait_result.value.signals = 5;
    TEST_CHECK(PlatformSignalWait(1u, PLATFORM_SIGNAL_WAIT_FOREVER, &observed) == PLATFORM_SIGNAL_RECEIVED);
    TEST_CHECK(observed == 5u && recorded_bits == 1u && recorded_timeout == PLATFORM_SIGNAL_WAIT_FOREVER);
    wait_result.status = osOK;
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_TIMEOUT && observed == 5u);
    wait_result.status = osEventTimeout;
    TEST_CHECK(PlatformSignalWait(1u, 20u, &observed) == PLATFORM_SIGNAL_TIMEOUT && observed == 5u);
    wait_result.status = osErrorISR;
    TEST_CHECK(PlatformSignalWait(1u, 20u, &observed) == PLATFORM_SIGNAL_ERROR && observed == 5u);
    return 0;
}
