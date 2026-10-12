#include "init_diagnostic.h"
#include "main.h"
#include <string.h>

volatile InitDiagnosticSnapshot_s init_diagnostic_snapshot;

static uint32_t EnterCritical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

static void ExitCritical(uint32_t previous)
{
    __set_PRIMASK(previous);
}

void InitDiagnosticBegin(InitDiagnosticStage_e stage)
{
    uint32_t previous = EnterCritical();
    if (!init_diagnostic_snapshot.failed)
        init_diagnostic_snapshot.stage = (uint8_t)stage;
    ExitCritical(previous);
}

void InitDiagnosticSetStage(InitDiagnosticStage_e stage)
{
    InitDiagnosticBegin(stage);
}

void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource_id)
{
    uint32_t previous = EnterCritical();
    if (init_diagnostic_snapshot.failure_count != UINT32_MAX)
        init_diagnostic_snapshot.failure_count++;
    if (!init_diagnostic_snapshot.failed)
    {
        init_diagnostic_snapshot.stage = (uint8_t)stage;
        init_diagnostic_snapshot.module = module;
        init_diagnostic_snapshot.reason = (uint8_t)reason;
        init_diagnostic_snapshot.resource_id = resource_id;
        init_diagnostic_snapshot.failed = 1u;
    }
    ExitCritical(previous);
}

void InitDiagnosticMarkReady(void)
{
    uint32_t previous = EnterCritical();
    if (!init_diagnostic_snapshot.failed)
        init_diagnostic_snapshot.stage = INIT_STAGE_READY;
    ExitCritical(previous);
}

void InitDiagnosticHeartbeat(void)
{
    uint32_t previous = EnterCritical();
    init_diagnostic_snapshot.heartbeat++;
    ExitCritical(previous);
}

uint8_t InitDiagnosticIsFailed(void)
{
    return init_diagnostic_snapshot.failed;
}

void InitDiagnosticGetSnapshot(InitDiagnosticSnapshot_s *snapshot)
{
    if (snapshot == NULL)
        return;
    uint32_t previous = EnterCritical();
    snapshot->failed = init_diagnostic_snapshot.failed;
    snapshot->stage = init_diagnostic_snapshot.stage;
    snapshot->module = init_diagnostic_snapshot.module;
    snapshot->reason = init_diagnostic_snapshot.reason;
    snapshot->resource_id = init_diagnostic_snapshot.resource_id;
    snapshot->reserved = init_diagnostic_snapshot.reserved;
    snapshot->failure_count = init_diagnostic_snapshot.failure_count;
    snapshot->heartbeat = init_diagnostic_snapshot.heartbeat;
    ExitCritical(previous);
}
