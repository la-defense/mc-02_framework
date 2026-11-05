/** @file platform_startup_wait_test.c
 * @brief 验证启动前等待的截止时间、计数回绕和停止计数兜底。
 */
#include "platform_startup_wait.h"
#include "test_check.h"
#include <stdint.h>

typedef struct {
    uint32_t cycles;
    uint32_t step;
    unsigned checks;
    unsigned ready_after;
} WaitModel;

static uint32_t ReadCycles(void *context)
{
    WaitModel *model = context;
    const uint32_t current = model->cycles;
    model->cycles += model->step; // 无符号加法模拟 32 位计数回绕。
    return current;
}

static uint8_t IsReady(void *context)
{
    WaitModel *model = context;
    ++model->checks;
    return (uint8_t)(model->ready_after != 0u && model->checks >= model->ready_after);
}

int main(void)
{
    WaitModel model = {.cycles = 100u, .step = 1u, .ready_after = 1u};
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 5u, 10u) == STARTUP_WAIT_READY);
    TEST_CHECK(model.checks == 1u);

    // 到期限仍未就绪时退出，不能让循环检查重设起点。
    model = (WaitModel){.cycles = UINT32_MAX - 3u, .step = 2u};
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 6u, 20u) == STARTUP_WAIT_TIMEOUT);
    TEST_CHECK(model.checks == 3u);

    // 周期计数停止时由次数上限退出，不依赖 HAL tick。
    model = (WaitModel){.cycles = 17u, .step = 0u};
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 6u, 4u) == STARTUP_WAIT_POLL_LIMIT);
    TEST_CHECK(model.checks == 4u);

    // 到期后才就绪不能获得成功结果。
    model = (WaitModel){.cycles = 0u, .step = 1u, .ready_after = 4u};
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 3u, 10u) == STARTUP_WAIT_TIMEOUT);
    TEST_CHECK(model.checks == 3u);
    TEST_CHECK(PlatformStartupWait(NULL, ReadCycles, &model, 3u, 10u) == STARTUP_WAIT_ARGUMENT);
    TEST_CHECK(PlatformStartupWait(IsReady, NULL, &model, 3u, 10u) == STARTUP_WAIT_ARGUMENT);
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 0u, 10u) == STARTUP_WAIT_ARGUMENT);
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, 3u, 0u) == STARTUP_WAIT_ARGUMENT);
    TEST_CHECK(PlatformStartupWait(IsReady, ReadCycles, &model, UINT32_MAX, 10u) == STARTUP_WAIT_ARGUMENT);
    return 0;
}
