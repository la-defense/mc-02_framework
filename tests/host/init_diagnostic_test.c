#include "test_check.h"
#include "init_diagnostic.h"
#include <stdint.h>

static uint32_t primask;
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t value) { primask = value; }

int main(void)
{
    InitDiagnosticSnapshot_s snapshot = {0};
    primask = 0u;
    InitDiagnosticBegin(INIT_STAGE_BSP);
    TEST_CHECK(primask == 0u);
    InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_GIMBAL, INIT_FAILURE_RESOURCE, 23u);
    InitDiagnosticReportFailure(INIT_STAGE_TASKS, 2u, INIT_FAILURE_TASK_CREATE, 1u);
    InitDiagnosticMarkReady();
    InitDiagnosticHeartbeat();
    InitDiagnosticGetSnapshot(&snapshot);
    TEST_CHECK(primask == 0u);
    TEST_CHECK(snapshot.failed == 1u);
    TEST_CHECK(snapshot.stage == INIT_STAGE_APPLICATION);
    TEST_CHECK(snapshot.module == INIT_MODULE_GIMBAL);
    TEST_CHECK(snapshot.reason == INIT_FAILURE_RESOURCE);
    TEST_CHECK(snapshot.resource_id == 23u);
    TEST_CHECK(snapshot.failure_count == 2u);
    TEST_CHECK(snapshot.heartbeat == 1u);
    TEST_CHECK(InitDiagnosticIsFailed() == 1u);
    return 0;
}
