/** @file referee_bench_platform_test.c
 * @brief 检查裁判裸板诊断通过框架时钟和临界区处理完成通知。
 * @note 测试替身不连接串口或真实裁判系统。
 */
#include "referee_bench.h"
#include "rm_referee.h"
#include "platform_sync.h"
#include "test_check.h"
#include <string.h>

static uint32_t now_ms;
static unsigned critical_depth;
static unsigned calls;
static uint8_t fresh;
static uint8_t complete_inline;
static RefereeTxRequest_t last_request;

uint32_t PlatformTimeNowMs(void) { return now_ms; }
void PlatformTaskCriticalEnter(void) { ++critical_depth; }
void PlatformTaskCriticalExit(void) { TEST_CHECK(critical_depth != 0u); --critical_depth; }
RefereeStatus_t RefereeInit(PlatformUartPort_t port, const RefereeConfig_t *config)
{
    TEST_CHECK(port == PLATFORM_UART_1 && config != NULL);
    return REFEREE_OK;
}
RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output)
{
    memset(output, 0, sizeof(*output));
    output->command[REF_CMD_ROBOT_STATE].state = fresh ? REFEREE_DATA_FRESH : REFEREE_DATA_EXPIRED;
    output->command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id = 3u;
    return REFEREE_OK;
}
RefereeSendStatus_t RefereeSubmit(const RefereeTxRequest_t *request, uint32_t *id)
{
    TEST_CHECK(critical_depth == 0u); // 提交不应持有任务临界区。
    ++calls;
    last_request = *request;
    *id = calls;
    if (complete_inline) {
        const USARTTxEvent_t event = {.id = calls, .started = 1u, .started_ms = now_ms,
                                      .result = USART_TX_COMPLETE};
        request->complete(request->context, &event); // 模拟提交返回前已收到完成通知。
    }
    return REFEREE_SEND_ACCEPTED;
}

int main(void)
{
    TEST_CHECK(RefereeBenchInit(PLATFORM_UART_1));
    now_ms = UINT32_MAX - 10u; // 起始时间靠近回绕边界。
    RefereeBenchTask();
    TEST_CHECK(calls == 0u); // 身份未更新时不发送。
    fresh = 1u;
    complete_inline = 1u;
    RefereeBenchTask();
    TEST_CHECK(calls == 1u && last_request.length == 112u && last_request.receiver_id == 4u);
    RefereeBenchTask();
    TEST_CHECK(calls == 2u); // 同步完成后请求可以再次提交。
    now_ms += 30000u;
    RefereeBenchTask();
    TEST_CHECK(calls == 3u && last_request.length == 2u && last_request.receiver_id == 0x103u);
    fresh = 0u;
    RefereeBenchTask();
    TEST_CHECK(calls == 3u); // 身份过期时禁止继续提交。
    fresh = 1u;
    now_ms += 45000u;
    RefereeBenchTask();
    TEST_CHECK(calls == 3u && critical_depth == 0u); // 到达总时限后停止。
    return 0;
}
