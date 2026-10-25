/**
 * @file init_diagnostic_test.c
 * @brief 检查生产启动诊断的首次失败和有界初始化事件记录。
 * @note 每个进程只模拟一次启动；通过替身读取毫秒时钟和中断状态，不连接硬件。
 */
#include "test_check.h"
#include "init_diagnostic.h"
#include <stdint.h>
#include <string.h>

static uint32_t primask;
static uint32_t now_ms;
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t value) { primask = value; }
uint32_t HAL_GetTick(void) { return now_ms; }

static void TestFirstFailure(void)
{
    InitDiagnosticSnapshot_s snapshot = {0};
    InitDiagnosticEventSnapshot_t events = {0};
    primask = 0u;
    now_ms = 0u;
    InitDiagnosticBegin(INIT_STAGE_BSP);
    TEST_CHECK(primask == 0u);
    now_ms = 10u;
    InitDiagnosticSetStage(INIT_STAGE_APPLICATION);
    now_ms = 11u;
    InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_GIMBAL, INIT_FAILURE_RESOURCE, 23u);
    now_ms = 12u;
    InitDiagnosticReportFailure(INIT_STAGE_TASKS, 2u, INIT_FAILURE_TASK_CREATE, 1u);
    InitDiagnosticMarkReady();
    InitDiagnosticHeartbeat();
    InitDiagnosticGetSnapshot(&snapshot);
    TEST_CHECK(InitDiagnosticGetEvents(&events) == 1u);
    TEST_CHECK(primask == 0u);
    TEST_CHECK(snapshot.failed == 1u);
    TEST_CHECK(snapshot.stage == INIT_STAGE_APPLICATION);
    TEST_CHECK(snapshot.module == INIT_MODULE_GIMBAL);
    TEST_CHECK(snapshot.reason == INIT_FAILURE_RESOURCE);
    TEST_CHECK(snapshot.resource_id == 23u);
    TEST_CHECK(snapshot.failure_count == 2u);
    TEST_CHECK(snapshot.heartbeat == 1u);
    TEST_CHECK(InitDiagnosticIsFailed() == 1u);
    TEST_CHECK(events.count == 4u);
    TEST_CHECK(events.events[0].sequence == 1u && events.events[0].timestamp_ms == 0u);
    TEST_CHECK(events.events[0].kind == INIT_DIAGNOSTIC_EVENT_STAGE_CHANGED);
    TEST_CHECK(events.events[1].sequence == 2u && events.events[1].timestamp_ms == 10u);
    TEST_CHECK(events.events[2].sequence == 3u && events.events[2].timestamp_ms == 11u);
    TEST_CHECK(events.events[2].kind == INIT_DIAGNOSTIC_EVENT_FAILURE);
    TEST_CHECK(events.events[2].module == INIT_MODULE_GIMBAL);
    TEST_CHECK(events.events[2].reason == INIT_FAILURE_RESOURCE);
    TEST_CHECK(events.events[2].severity == INIT_DIAGNOSTIC_SEVERITY_ERROR);
    TEST_CHECK(events.events[2].resource_id == 23u);
    TEST_CHECK(events.events[3].sequence == 4u && events.events[3].timestamp_ms == 12u);
    TEST_CHECK(events.events[3].module == 2u && events.events[3].reason == INIT_FAILURE_TASK_CREATE);
    TEST_CHECK(events.events[3].severity == INIT_DIAGNOSTIC_SEVERITY_ERROR);
}

static void TestEventHistory(void)
{
    InitDiagnosticEventSnapshot_t events = {0};
    InitDiagnosticSnapshot_s failure = {0};
    now_ms = 0u;
    InitDiagnosticBegin(INIT_STAGE_BSP);
    for (uint32_t index = 0u; index < 20u; ++index)
    {
        now_ms = 100u + index;
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION,
                                    INIT_MODULE_ROBOT,
                                    INIT_FAILURE_RESOURCE,
                                    (uint16_t)(index + 1u));
    }

    TEST_CHECK(InitDiagnosticGetEvents(&events) == 1u);
    TEST_CHECK(events.count == INIT_DIAGNOSTIC_EVENT_CAPACITY);
    TEST_CHECK(events.total_count == 21u);
    TEST_CHECK(events.events[0].sequence == 6u);
    TEST_CHECK(events.events[0].timestamp_ms == 104u);
    TEST_CHECK(events.events[0].resource_id == 5u);
    TEST_CHECK(events.events[INIT_DIAGNOSTIC_EVENT_CAPACITY - 1u].sequence == 21u);
    TEST_CHECK(events.events[INIT_DIAGNOSTIC_EVENT_CAPACITY - 1u].timestamp_ms == 119u);
    TEST_CHECK(InitDiagnosticGetEvents(&events) == 1u);
    TEST_CHECK(events.events[0].sequence == 6u && events.total_count == 21u);
    TEST_CHECK(InitDiagnosticGetEvents(NULL) == 0u);
    InitDiagnosticGetSnapshot(&failure);
    TEST_CHECK(failure.failed == 1u && failure.resource_id == 1u);
    TEST_CHECK(failure.failure_count == 20u);
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "history") == 0)
        TestEventHistory();
    else
        TestFirstFailure();
    return 0;
}
