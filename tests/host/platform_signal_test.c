/** @file platform_signal_test.c
 * @brief 验证任务信号平台适配的参数、合并设置与等待结果。
 * @note 用 CMSIS 替身检查调用边界，不模拟真实中断或调度时限。
 */
#include "platform_signal.h"
#include "cmsis_os.h"
#include "test_check.h"

static uint32_t calls_set, calls_wait, recorded_bits, recorded_wait_bits, recorded_timeout;
static int32_t set_result;
static osEvent wait_result;
static uint8_t model_enabled, model_pending;
static uint32_t model_bits, inject_after_wait;
static osThreadId current_thread = (void *)1;

osThreadId osThreadGetId(void)
{
    return current_thread; // 正常情况返回当前任务，也可模拟后端无法取得句柄。
}

int32_t osSignalSet(osThreadId thread, int32_t signals)
{
    TEST_CHECK(thread == (void *)1);
    ++calls_set;
    recorded_bits = (uint32_t)signals;
    if (model_enabled)
    {
        const uint32_t previous = model_bits; // 保存设置前的值供 CMSIS 返回。
        model_bits |= (uint32_t)signals; // FreeRTOS 的 eSetBits 合并新位。
        model_pending = 1u; // 设置通知后，下一次等待不能继续睡眠。
        return (int32_t)previous;
    }
    return set_result;
}
osEvent osSignalWait(int32_t signals, uint32_t milliseconds)
{
    ++calls_wait;
    recorded_wait_bits = (uint32_t)signals;
    recorded_timeout = milliseconds;
    if (model_enabled)
    {
        osEvent result = {0}; // 模拟已挂起标志与通知数值分别保存。
        if (!model_pending)
            return result; // 没有待处理通知时，立即检查返回 osOK。
        result.status = osEventSignal;
        result.value.signals = (int32_t)model_bits; // 交付清理之前的全部位。
        model_bits &= ~(uint32_t)signals; // 只清除指定的通知位。
        model_pending = 0u; // FreeRTOS 返回时总会清除待处理标志。
        if (inject_after_wait != 0u)
        {
            model_bits |= inject_after_wait; // 模拟返回后、中断发送的新信号。
            model_pending = 1u; // 新中断再次设置待处理标志。
            inject_after_wait = 0u;
        }
        return result;
    }
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
    set_result = 0;
    wait_result.status = osEventSignal;
    wait_result.value.signals = 5;
    TEST_CHECK(PlatformSignalWait(1u, PLATFORM_SIGNAL_WAIT_FOREVER, &observed) == PLATFORM_SIGNAL_RECEIVED);
    TEST_CHECK(observed == 5u && recorded_wait_bits == 1u && recorded_timeout == PLATFORM_SIGNAL_WAIT_FOREVER);
    TEST_CHECK(recorded_bits == 4u); // 未清除的位须重新标记为待处理。
    set_result = (int32_t)0x80000000u;
    observed = 0x1234u;
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_ERROR && observed == 0x1234u);
    set_result = 0;
    current_thread = NULL;
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_ERROR && observed == 0x1234u);
    current_thread = (void *)1;
    wait_result.status = osOK;
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_TIMEOUT && observed == 0x1234u);
    wait_result.status = osEventTimeout;
    TEST_CHECK(PlatformSignalWait(1u, 20u, &observed) == PLATFORM_SIGNAL_TIMEOUT && observed == 0x1234u);
    wait_result.status = osErrorISR;
    TEST_CHECK(PlatformSignalWait(1u, 20u, &observed) == PLATFORM_SIGNAL_ERROR && observed == 0x1234u);

    // 清除一个位后，其他位必须继续可被下一次等待取到。
    model_enabled = 1u;
    TEST_CHECK(PlatformSignalSet((void *)1, 3u));
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_RECEIVED && observed == 3u);
    TEST_CHECK(PlatformSignalWait(2u, 0u, &observed) == PLATFORM_SIGNAL_RECEIVED && observed == 2u);
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_TIMEOUT);

    // 等待返回和补发之间若有新中断，合并时不能覆盖新到的位。
    TEST_CHECK(PlatformSignalSet((void *)1, 3u));
    inject_after_wait = 4u;
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_RECEIVED && observed == 3u);
    TEST_CHECK(PlatformSignalWait(6u, 0u, &observed) == PLATFORM_SIGNAL_RECEIVED && observed == 6u);
    TEST_CHECK(PlatformSignalWait(1u, 0u, &observed) == PLATFORM_SIGNAL_TIMEOUT);
    return 0;
}
