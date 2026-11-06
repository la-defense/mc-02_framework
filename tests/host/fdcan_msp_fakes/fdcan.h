/** @file fdcan.h
 * @brief 供主机编译生产 FDCAN MSP 初始化的最小 HAL 替身。
 */
#ifndef MC02_TEST_FDCAN_MSP_H
#define MC02_TEST_FDCAN_MSP_H
#include <stdint.h>

typedef struct { uint32_t unused; } FDCAN_GlobalTypeDef;
typedef struct {
    uint32_t FrameFormat, Mode, AutoRetransmission, TransmitPause, ProtocolException;
    uint32_t NominalPrescaler, NominalSyncJumpWidth, NominalTimeSeg1, NominalTimeSeg2;
    uint32_t DataPrescaler, DataSyncJumpWidth, DataTimeSeg1, DataTimeSeg2;
    uint32_t MessageRAMOffset, StdFiltersNbr, ExtFiltersNbr, RxFifo0ElmtsNbr;
    uint32_t RxFifo0ElmtSize, RxFifo1ElmtsNbr, RxFifo1ElmtSize, RxBuffersNbr;
    uint32_t RxBufferSize, TxEventsNbr, TxBuffersNbr, TxFifoQueueElmtsNbr;
    uint32_t TxFifoQueueMode, TxElmtSize;
} FDCAN_InitTypeDef;
typedef struct { FDCAN_GlobalTypeDef *Instance; FDCAN_InitTypeDef Init; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t PeriphClockSelection, FdcanClockSelection; } RCC_PeriphCLKInitTypeDef;
extern FDCAN_GlobalTypeDef fake_fdcan_devices[3];
#define FDCAN1 (&fake_fdcan_devices[0])
#define FDCAN2 (&fake_fdcan_devices[1])
#define FDCAN3 (&fake_fdcan_devices[2])
#define FDCAN_FRAME_CLASSIC 0u
#define FDCAN_MODE_NORMAL 0u
#define FDCAN_DATA_BYTES_8 4u
#define FDCAN_DATA_BYTES_64 18u
#define FDCAN_TX_FIFO_OPERATION 0u
#define ENABLE 1u
#define DISABLE 0u
#define HAL_OK 0u
#define HAL_ERROR 1u
#define RCC_PERIPHCLK_FDCAN 1u
#define RCC_FDCANCLKSOURCE_PLL 2u
#define GPIO_PIN_0 (1u << 0)
#define GPIO_PIN_1 (1u << 1)
#define GPIO_PIN_5 (1u << 5)
#define GPIO_PIN_6 (1u << 6)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_MODE_AF_PP 1u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_AF9_FDCAN1 9u
#define GPIO_AF9_FDCAN2 9u
#define GPIO_AF5_FDCAN3 5u
#define GPIOD ((void *)1u)
#define GPIOB ((void *)2u)
enum { FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn, FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn,
       FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn };
void FakeFdcanClockEnable(void);
void FakeFdcanClockDisable(void);
#define __HAL_RCC_FDCAN_CLK_ENABLE() FakeFdcanClockEnable()
#define __HAL_RCC_FDCAN_CLK_DISABLE() FakeFdcanClockDisable()
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
uint32_t HAL_FDCAN_Init(FDCAN_HandleTypeDef *handle);
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *handle);
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *handle);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
void HAL_GPIO_DeInit(void *port, uint32_t pins);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void Error_Handler(void);
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);
#endif
