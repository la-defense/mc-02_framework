/** @file referee_tx_budget.c
 * @brief 纯C共享发送额度；Gate检查34ms及滚动1000ms，Observe保留已占用的额度。
 * @note 无共享状态、时钟读取或分配；由裁判模块在短临界区串行调用。
 */
#include "referee_tx_private.h"
uint8_t RefereeTxRobotId(uint16_t id)
{
    return (uint8_t)((id >= 1u && id <= 9u) || (id >= 101u && id <= 109u));
}
USARTTxGateResult_t RefereeBudgetGate(RefereeTxBudget *b, uint32_t id, uint16_t receiver, uint16_t bytes,
                                      uint32_t now)
{
    b->state.wait_reason = REFEREE_TX_WAIT_NONE;
    // 所有子命令共用启动间隔；切身份或重新初始化UI不能绕过这份历史。
    if (b->state.has_start && (uint32_t)(now - b->state.last_start_ms) < 34u)
    {
        b->state.wait_reason = REFEREE_TX_WAIT_RATE;
    }
    else if (RefereeTxRobotId(receiver))
    {
        uint32_t total = bytes;
        uint32_t cap =
            receiver == 7u || receiver == 9u || receiver == 107u || receiver == 109u ? 5120u : 3720u;
        for (unsigned i = 0u; i < 30u; ++i)
            if (b->history[i].used && b->history[i].receiver == receiver &&
                (uint32_t)(now - b->history[i].at_ms) < 1000u)
                total += b->history[i].bytes;
        if (total > cap)
            b->state.wait_reason = REFEREE_TX_WAIT_BYTES;
    }
    if (b->state.wait_reason != REFEREE_TX_WAIT_NONE)
    {
        ++b->state.gate_deferrals;
        return USART_TX_DEFER;
    }
    b->candidate_id = id;
    b->candidate_receiver = receiver;
    b->candidate_bytes = bytes;
    return USART_TX_ALLOW;
}
void RefereeBudgetObserve(RefereeTxBudget *b, const USARTTxEvent_t *event)
{
    b->state.wait_reason = REFEREE_TX_WAIT_NONE;
    if (event->result == USART_TX_CANCELLED)
        ++b->state.cancelled;
    else if (event->result == USART_TX_EXPIRED)
        ++b->state.expired;
    else if (event->result != USART_TX_COMPLETE)
        ++b->state.errors;
    if (event->id != b->candidate_id)
        return;
    // HAL启动失败可能已触及硬件，按整帧保守扣费；排队超期或取消不扣费。
    if (event->started || event->result == USART_TX_FAILED || event->result == USART_TX_ISOLATED)
    {
        uint32_t at = event->started ? event->started_ms : event->completed_ms;
        b->state.last_start_ms = at;
        b->state.has_start = 1u;
        for (unsigned i = 0u; i < 30u; ++i)
        {
            if (!b->history[i].used || (uint32_t)(at - b->history[i].at_ms) >= 1000u)
            {
                b->history[i] = (RefereeTxCharge){at, b->candidate_receiver, b->candidate_bytes, 1u};
                break;
            }
        }
    }
    b->candidate_id = 0u;
}
