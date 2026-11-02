/**
 * @file can_tx_fake_api.h
 * @brief 多模块主机测试共享的 CAN 复制队列接口声明。
 * @note 仅由测试替身包含；具体发送结果由各测试进程自行实现，不进入固件。
 */
#ifndef MC02_TEST_CAN_TX_FAKE_API_H
#define MC02_TEST_CAN_TX_FAKE_API_H

#include "../../bsp/can/can_tx.h"

typedef enum
{
    CAN_TX_CANCEL_OK = 1
} CAN_TxCancelStatus_t;

#endif
