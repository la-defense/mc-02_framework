/** @file referee_bench.c
 * @brief REF-03裸板有限负载及只读完成记录；通过生产共享入口，不创建任务。
 * @note 默认编译为空；诊断仅bench_safe、禁热、执行器断开，不能验证真实客户端显示。
 */
#if MC02_BENCH_REFEREE_DIAGNOSTICS
#if !defined(MC02_PROFILE_BENCH_SAFE)
#error "Referee diagnostics require bench_safe"
#endif
#include "referee_bench.h"
#include "rm_referee.h"
#include "cmsis_os.h"
#include <string.h>
typedef struct
{
    uint32_t id, submitted_ms, started_ms, completed_ms;
    uint32_t result, started, length_kind;
} RefereeBenchRecord;
typedef struct
{
    uint32_t magic, version, begin_ms, phase;
    uint32_t submitted, busy, rejected, write_count;
    RefereeBenchRecord records[64];
} RefereeBenchDiagnostic;
// write_count奇数表示更新中，偶数除以2为完成记录数；只读工具按读取前的已发布数取旧记录，并检查读后没有绕环覆盖。
_Static_assert(sizeof(RefereeBenchDiagnostic) == 1824u, "Referee trace layout changed");
volatile RefereeBenchDiagnostic referee_bench_diag;
typedef struct
{
    uint8_t kind, pending, has_start;
    uint16_t length, receiver;
    uint32_t last_start_ms;
} BenchRequest;
static BenchRequest requests[2] = {{.kind = 0u, .length = 127u}, {.kind = 1u, .length = 17u}};
static uint8_t full_content[112];
static const uint8_t ui_noop[2] = {0u, 0u};
static void BenchComplete(void *context, const USARTTxEvent_t *event)
{
    BenchRequest *request = context;
    taskENTER_CRITICAL();
    request->pending = 0u;
    if (event->started)
    {
        request->has_start = 1u;
        request->last_start_ms = event->started_ms;
    }
    uint32_t count = referee_bench_diag.write_count / 2u;
    referee_bench_diag.write_count = count * 2u + 1u;
    __DMB();
    RefereeBenchRecord record = {event->id,
                                 event->submitted_ms,
                                 event->started_ms,
                                 event->completed_ms,
                                 (uint32_t)event->result,
                                 event->started,
                                 (uint32_t)request->length | ((uint32_t)request->kind << 8u) |
                                     ((uint32_t)request->receiver << 16u)};
    referee_bench_diag.records[count % 64u] = record;
    __DMB();
    referee_bench_diag.write_count = count * 2u + 2u;
    taskEXIT_CRITICAL();
}
uint8_t RefereeBenchInit(UART_HandleTypeDef *handle)
{
    memset((void *)&referee_bench_diag, 0, sizeof(referee_bench_diag));
    referee_bench_diag.magic = 0x52454633u;
    referee_bench_diag.version = 1u;
    for (unsigned i = 0u; i < sizeof(full_content); ++i)
        full_content[i] = (uint8_t)i;
    // 仅供裸板测试模型使用的3→4方向，不代表任何赛项官方许可。
    const RefereeConfig_t config = {.timeout_10hz_ms = 300u, .timeout_3hz_ms = 1000u,
        .timeout_1hz_ms = 3000u, .timeout_map_ms = 3000u,
        .interaction = {REFEREE_COMPETITION_RMUC_2026, REFEREE_PROTOCOL_2026_V2, 3u, 1u << 3u, 0u}};
    return (uint8_t)(RefereeInit(handle, &config) == REFEREE_OK);
}
void RefereeBenchTask(void)
{
    static RefereeSnapshot_t snapshot;
    if (RefereeGetSnapshot(&snapshot) != REFEREE_OK)
        return;
    const RefereeCommandSnapshot_t *robot = &snapshot.command[REF_CMD_ROBOT_STATE];
    uint32_t now = HAL_GetTick();
    if (referee_bench_diag.phase == 0u)
    {
        if (robot->state != REFEREE_DATA_FRESH)
            return;
        referee_bench_diag.begin_ms = now;
        referee_bench_diag.phase = 1u;
    }
    uint32_t age = (uint32_t)(now - referee_bench_diag.begin_ms);
    if (age >= 75000u)
    {
        referee_bench_diag.phase = 3u;
        return;
    }
    referee_bench_diag.phase = age < 30000u ? 1u : 2u;
    // 身份停更时不添加报文；已经排队的旧身份仍由生产启动检查取消。
    if (robot->state != REFEREE_DATA_FRESH)
        return;
    uint16_t sender = robot->data.robot_state.robot_id;
    uint8_t kind = 0u;
    taskENTER_CRITICAL();
    if (age >= 30000u && !requests[1].pending &&
        (!requests[1].has_start || (uint32_t)(now - requests[1].last_start_ms) >= 115u))
        kind = 1u;
    BenchRequest *r = &requests[kind];
    uint8_t pending = r->pending;
    taskEXIT_CRITICAL();
    if (pending)
        return;
    uint16_t receiver = kind ? (uint16_t)(sender + 0x100u) : (sender >= 101u ? 104u : 4u);
    RefereeTxRequest_t request = {receiver,
                                  kind ? 0x0100u : 0x0200u,
                                  kind ? ui_noop : full_content,
                                  kind ? 2u : 112u,
                                  100u,
                                  BenchComplete,
                                  NULL,
                                  r};
    r->receiver = receiver;
    uint32_t id;
    // 先标记，防止高优先级发送任务在Submit返回后立刻通知并被旧赋值覆盖。
    taskENTER_CRITICAL();
    r->pending = 1u;
    taskEXIT_CRITICAL();
    RefereeSendStatus_t result = RefereeSubmit(&request, &id);
    if (result != REFEREE_SEND_ACCEPTED)
    {
        taskENTER_CRITICAL();
        r->pending = 0u;
        taskEXIT_CRITICAL();
        if (result == REFEREE_SEND_BUSY)
            ++referee_bench_diag.busy;
        else
            ++referee_bench_diag.rejected;
    }
    else
        ++referee_bench_diag.submitted;
}
#endif
