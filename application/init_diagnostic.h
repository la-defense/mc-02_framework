#ifndef MC02_INIT_DIAGNOSTIC_H
#define MC02_INIT_DIAGNOSTIC_H

#include <stdint.h>

typedef enum
{
    INIT_STAGE_RESET = 0,
    INIT_STAGE_BSP,
    INIT_STAGE_APPLICATION,
    INIT_STAGE_TASKS,
    INIT_STAGE_FREERTOS,
    INIT_STAGE_READY,
} InitDiagnosticStage_e;

typedef enum
{
    INIT_FAILURE_NONE = 0,
    INIT_FAILURE_RESOURCE,
    INIT_FAILURE_TASK_CREATE,
    INIT_FAILURE_SCHEDULER_RETURNED,
    INIT_FAILURE_ALGORITHM_MEMORY,
    INIT_FAILURE_DEPENDENCY,
} InitDiagnosticReason_e;

typedef enum
{
    INIT_MODULE_ROBOT = 1,
    INIT_MODULE_REMOTE,
    INIT_MODULE_VISION,
    INIT_MODULE_GIMBAL,
    INIT_MODULE_SHOOTER,
    INIT_MODULE_CHASSIS,
    INIT_MODULE_INS,
    INIT_MODULE_TASKS,
    INIT_MODULE_FREERTOS,
    INIT_MODULE_BUZZER,
} InitDiagnosticModule_e;

typedef enum
{
    INIT_RESOURCE_TASK_INS = 1,
    INIT_RESOURCE_TASK_MOTOR,
    INIT_RESOURCE_TASK_DAEMON,
    INIT_RESOURCE_TASK_ROBOT,
    INIT_RESOURCE_TASK_UI,
    INIT_RESOURCE_TASK_LCD,
    INIT_RESOURCE_TASK_DEFAULT,
} InitDiagnosticResource_e;

typedef struct
{
    uint8_t failed;
    uint8_t stage;
    uint8_t module;
    uint8_t reason;
    uint16_t resource_id;
    uint16_t reserved;
    uint32_t failure_count;
    uint32_t heartbeat;
} InitDiagnosticSnapshot_s;

/* Exported fixed-layout snapshot for bounded, read-only SWD inspection. */
extern volatile InitDiagnosticSnapshot_s init_diagnostic_snapshot;

void InitDiagnosticBegin(InitDiagnosticStage_e stage);
void InitDiagnosticSetStage(InitDiagnosticStage_e stage);
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource_id);
void InitDiagnosticMarkReady(void);
void InitDiagnosticHeartbeat(void);
uint8_t InitDiagnosticIsFailed(void);
void InitDiagnosticGetSnapshot(InitDiagnosticSnapshot_s *snapshot);

#endif
