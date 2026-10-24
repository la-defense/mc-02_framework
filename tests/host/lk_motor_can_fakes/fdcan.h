/**
 * @file fdcan.h
 * @brief LK 与生产 CAN 集成回归使用的 FDCAN HAL 最小替身。
 * 仅在主机测试目标中代替 HAL 类型和声明，不进入固件。
 */
#ifndef MC02_LK_CAN_TEST_FDCAN_H
#define MC02_LK_CAN_TEST_FDCAN_H

#include <stdint.h>

#define HAL_OK 0u
#define HAL_ERROR 1u
typedef uint32_t HAL_StatusTypeDef;
#define HAL_FDCAN_STATE_READY 1u
#define HAL_FDCAN_STATE_BUSY 2u

#define FDCAN_DLC_BYTES_0 0u
#define FDCAN_DLC_BYTES_1 1u
#define FDCAN_DLC_BYTES_2 2u
#define FDCAN_DLC_BYTES_3 3u
#define FDCAN_DLC_BYTES_4 4u
#define FDCAN_DLC_BYTES_5 5u
#define FDCAN_DLC_BYTES_6 6u
#define FDCAN_DLC_BYTES_7 7u
#define FDCAN_DLC_BYTES_8 8u

#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1u << 0)
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST (1u << 1)
#define FDCAN_IT_RX_FIFO0_FULL (1u << 2)
#define FDCAN_IT_RX_FIFO0_WATERMARK (1u << 3)
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE (1u << 4)
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST (1u << 5)
#define FDCAN_IT_RX_FIFO1_FULL (1u << 6)
#define FDCAN_IT_RX_FIFO1_WATERMARK (1u << 7)
#define FDCAN_IT_TX_COMPLETE (1u << 8)
#define FDCAN_IT_TX_ABORT_COMPLETE (1u << 9)

#define FDCAN_RX_FIFO0 0u
#define FDCAN_RX_FIFO1 1u
#define FDCAN_RX_FIFO_OVERWRITE 0u
#define FDCAN_REJECT 0u
#define FDCAN_REJECT_REMOTE 0u
#define FDCAN_FILTER_DUAL 0u
#define FDCAN_FILTER_TO_RXFIFO0 0u
#define FDCAN_FILTER_TO_RXFIFO1 1u
#define FDCAN_STANDARD_ID 0u
#define FDCAN_EXTENDED_ID 1u
#define FDCAN_DATA_FRAME 0u
#define FDCAN_MODE_NORMAL 0u
#define FDCAN_MODE_EXTERNAL_LOOPBACK 1u
#define FDCAN_CCCR_INIT (1u << 0)
#define FDCAN_FILTER_MASK 0u
#define FDCAN_ESI_ACTIVE 0u
#define FDCAN_BRS_OFF 0u
#define FDCAN_CLASSIC_CAN 0u
#define FDCAN_NO_TX_EVENTS 0u

typedef struct
{
    uint32_t CCCR;
    uint32_t IE;
    uint32_t ILE;
    uint32_t ILS;
    uint32_t TEST;
    uint32_t PSR;
    uint32_t ECR;
    uint32_t RXF0S;
    uint32_t GFC;
    uint32_t RXF0C;
} FDCAN_GlobalTypeDef;

typedef struct
{
    uint32_t StdFiltersNbr;
    uint32_t Mode;
} FDCAN_InitTypeDef;

typedef struct
{
    FDCAN_InitTypeDef Init;
    FDCAN_GlobalTypeDef *Instance;
    uint32_t State;
} FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
    uint32_t IdType;
    uint32_t IsCalibrationMsg;
} FDCAN_FilterTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t DataLength;
    uint32_t RxFrameType;
    uint32_t IdType;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t BusOff;
    uint32_t ErrorPassive;
} FDCAN_ProtocolStatusTypeDef;

typedef struct
{
    uint32_t TxErrorCnt;
    uint32_t RxErrorCnt;
} FDCAN_ErrorCountersTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

uint32_t HAL_GetTick(void);
uint32_t HAL_FDCAN_ConfigRxFifoOverwrite(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t mode);
uint32_t HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t nonmatching_std,
                                      uint32_t nonmatching_ext, uint32_t remote_std, uint32_t remote_ext);
uint32_t HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle);
uint32_t HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle);
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle);
uint32_t HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t buffer_indexes);
uint32_t HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, FDCAN_FilterTypeDef *filter);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle);
uint32_t HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                       FDCAN_TxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *handle, FDCAN_ProtocolStatusTypeDef *status);
uint32_t HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *handle, FDCAN_ErrorCountersTypeDef *counters);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo);
uint32_t HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
                                FDCAN_RxHeaderTypeDef *header, uint8_t *data);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t flags);
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle);
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *handle, uint32_t indexes);
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *handle, uint32_t indexes);

#endif
