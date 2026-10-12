#include "init_diagnostic.h"
#include <assert.h>
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
    assert(primask == 0u);
    InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_GIMBAL, INIT_FAILURE_RESOURCE, 23u);
    InitDiagnosticReportFailure(INIT_STAGE_TASKS, 2u, INIT_FAILURE_TASK_CREATE, 1u);
    InitDiagnosticMarkReady();
    InitDiagnosticHeartbeat();
    InitDiagnosticGetSnapshot(&snapshot);
    assert(primask == 0u);
    assert(snapshot.failed == 1u);
    assert(snapshot.stage == INIT_STAGE_APPLICATION);
    assert(snapshot.module == INIT_MODULE_GIMBAL);
    assert(snapshot.reason == INIT_FAILURE_RESOURCE);
    assert(snapshot.resource_id == 23u);
    assert(snapshot.failure_count == 2u);
    assert(snapshot.heartbeat == 1u);
    assert(InitDiagnosticIsFailed() == 1u);
    return 0;
}
