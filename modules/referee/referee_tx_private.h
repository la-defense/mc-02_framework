/** @file referee_tx_private.h
 * @brief 裁判额度计算内部数据；调用者提供存储与毫秒值，并串行访问，不读硬件。
 */
#ifndef MC02_REFEREE_TX_PRIVATE_H
#define MC02_REFEREE_TX_PRIVATE_H
#include "referee_tx.h"
typedef struct
{
    uint32_t at_ms;
    uint16_t receiver, bytes;
    uint8_t used;
} RefereeTxCharge;
typedef struct
{
    RefereeTxCharge history[30];
    uint32_t candidate_id;
    uint16_t candidate_receiver, candidate_bytes;
    RefereeTxState_t state;
} RefereeTxBudget;
// budget在启动时清零；检查只处理固定30条记录，DEFER不扣额度。
USARTTxGateResult_t RefereeBudgetGate(RefereeTxBudget *budget, uint32_t id, uint16_t receiver, uint16_t bytes,
                                      uint32_t now_ms);
// 在本请求业务回调前执行；按UART实际启动时刻计费，启动失败按完成时刻保守计费。
void RefereeBudgetObserve(RefereeTxBudget *budget, const USARTTxEvent_t *event);
// 协议允许的机器人编号1～9、101～109，不含基地和前哨站。
uint8_t RefereeTxRobotId(uint16_t id);
#endif
