/**
 * @file init_diagnostic.c
 * @brief 实现启动诊断的阶段更新、首次失败记录和查询，接口用法见 init_diagnostic.h。
 * 修改或复制记录时暂时关闭普通中断，避免处理到一半时被其他代码改写。
 */
#include "init_diagnostic.h"
#include "main.h"
#include <string.h>

// 普通全局变量在启动时清零，本次启动中的失败会一直保留到下次启动。
volatile InitDiagnosticSnapshot_s init_diagnostic_snapshot;

// 保存原中断开关状态，再暂时关闭普通中断。
static uint32_t EnterCritical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

// 恢复调用前的状态，调用前已关中断时继续保持关闭。
static void ExitCritical(uint32_t previous)
{
    __set_PRIMASK(previous);
}

void InitDiagnosticBegin(InitDiagnosticStage_e stage)
{
    uint32_t previous = EnterCritical();
    // 一旦记录失败，就保留当时的阶段，方便追查最先出错的位置。
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
    // 计数到上限后保持不变，避免再次加 1 使它回到 0。
    if (init_diagnostic_snapshot.failure_count != UINT32_MAX)
        init_diagnostic_snapshot.failure_count++;
    // 只在第一次失败时填写详情；后续报告仍计数，保留最先出错的线索。
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
    // 后续模块完成初始化，也不能覆盖前面已经记录的失败阶段。
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
    // 没有可写入的位置时跳过本次读取。
    if (snapshot == NULL)
        return;
    // 一次复制完整记录，防止普通中断在两个字段的读取之间改写数据。
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
