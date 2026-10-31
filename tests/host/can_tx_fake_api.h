/**
 * @file can_tx_fake_api.h
 * @brief 多模块主机测试共享的 CAN 复制队列接口声明。
 * @note 仅由测试替身包含；具体发送结果由各测试进程自行实现，不进入固件。
 */
#ifndef MC02_TEST_CAN_TX_FAKE_API_H
#define MC02_TEST_CAN_TX_FAKE_API_H

#include <stddef.h>
#include <stdint.h>

#define CAN_TX_MAX_BATCH_FRAMES 13u

typedef enum
{
    CAN_TX_ID_STANDARD = 0,
    CAN_TX_ID_EXTENDED = 1
} CAN_TxIdType_t;

typedef struct
{
    uint32_t identifier;
    CAN_TxIdType_t identifier_type;
    const uint8_t *data;
    uint8_t length;
} CAN_TxFrame_t;

typedef enum
{
    CAN_TX_PRIORITY_URGENT = 0,
    CAN_TX_PRIORITY_CONTROL = 1,
    CAN_TX_PRIORITY_NORMAL = 2,
    CAN_TX_PRIORITY_BACKGROUND = 3
} CAN_TxPriority_t;

typedef enum
{
    CAN_TX_SUBMIT_ACCEPTED = 0,
    CAN_TX_SUBMIT_COALESCED,
    CAN_TX_SUBMIT_INVALID,
    CAN_TX_SUBMIT_NOT_READY,
    CAN_TX_SUBMIT_FULL,
    CAN_TX_SUBMIT_BANDWIDTH,
    CAN_TX_SUBMIT_CONTEXT
} CAN_TxSubmitStatus_t;

typedef enum
{
    CAN_TX_RESULT_QUEUED = 0,
    CAN_TX_RESULT_ACTIVE,
    CAN_TX_RESULT_IN_FLIGHT,
    CAN_TX_RESULT_COMPLETE,
    CAN_TX_RESULT_FAILED,
    CAN_TX_RESULT_EXPIRED,
    CAN_TX_RESULT_CANCELLED,
    CAN_TX_RESULT_SUPERSEDED
} CAN_TxResultState_t;

typedef struct
{
    uint32_t request_id;
    CAN_TxResultState_t state;
    uint8_t frame_count;
    uint8_t accepted_frames;
    uint8_t completed_frames;
    uint8_t pending_frames;
    uint32_t last_hal_error;
} CAN_TxResult_t;

typedef enum
{
    CAN_TX_CANCEL_OK = 1
} CAN_TxCancelStatus_t;

#endif
