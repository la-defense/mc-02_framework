/**
 * @file stm32h7xx_hal.h
 * @brief 提供 WS2812 初始化测试需要的最小 HAL 类型和常量。
 * @note 测试只记录 GPIO 配置，不模拟 SPI 位时序或板上电平。
 */
#ifndef MC02_WS2812_TEST_HAL_H
#define MC02_WS2812_TEST_HAL_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum
{
    HAL_SPI_STATE_RESET = 0,
    HAL_SPI_STATE_READY,
    HAL_SPI_STATE_BUSY_TX,
    HAL_SPI_STATE_ERROR
} HAL_SPI_StateTypeDef;

typedef struct
{
    uint32_t ODR;
} GPIO_TypeDef;

typedef struct { uint32_t CR1, CFG1, IER; } SPI_TypeDef;
typedef struct { uint32_t CCR; } BDMA_Channel_TypeDef;
typedef struct { uint32_t CCR; } SCB_Type;
typedef struct
{
    uint32_t Request, Direction, PeriphInc, MemInc, PeriphDataAlignment;
    uint32_t MemDataAlignment, Mode, Priority;
} DMA_InitTypeDef;
typedef struct
{
    void *Instance;
    DMA_InitTypeDef Init;
    void *Parent;
    uint32_t State;
    void (*XferAbortCallback)(void *);
} DMA_HandleTypeDef;

extern GPIO_TypeDef fake_gpioa;
extern SPI_TypeDef fake_spi6;
extern BDMA_Channel_TypeDef fake_bdma0;
extern SCB_Type fake_scb;
#define GPIOA (&fake_gpioa)
#define SPI6 (&fake_spi6)
#define BDMA_Channel0 (&fake_bdma0)
#define SCB (&fake_scb)
#define SCB_CCR_DC_Msk 1u

typedef struct
{
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
        BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial,
        NSSPMode, NSSPolarity, FifoThreshold, TxCRCInitializationPattern,
        RxCRCInitializationPattern, MasterSSIdleness, MasterInterDataIdleness,
        MasterReceiverAutoSusp, MasterKeepIOState, IOSwap;
} SPI_InitTypeDef;

typedef struct
{
    SPI_TypeDef *Instance;
    SPI_InitTypeDef Init;
    HAL_SPI_StateTypeDef State;
    DMA_HandleTypeDef *hdmatx;
} SPI_HandleTypeDef;

typedef struct
{
    uint32_t Pin, Mode, Pull, Speed, Alternate;
} GPIO_InitTypeDef;

typedef struct
{
    uint32_t PeriphClockSelection, Spi6ClockSelection;
} RCC_PeriphCLKInitTypeDef;

#define SPI_POLARITY_LOW 0u
#define SPI_PHASE_2EDGE 1u
#define SPI_NSS_SOFT 0u
#define SPI_BAUDRATEPRESCALER_4 4u
#define SPI_FIRSTBIT_MSB 0u
#define SPI_MODE_MASTER 1u
#define SPI_DIRECTION_2LINES_TXONLY 2u
#define SPI_DATASIZE_8BIT 8u
#define SPI_TIMODE_DISABLE 0u
#define SPI_CRCCALCULATION_DISABLE 0u
#define SPI_NSS_PULSE_ENABLE 1u
#define SPI_NSS_POLARITY_LOW 0u
#define SPI_FIFO_THRESHOLD_01DATA 1u
#define SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN 0u
#define SPI_MASTER_SS_IDLENESS_00CYCLE 0u
#define SPI_MASTER_INTERDATA_IDLENESS_00CYCLE 0u
#define SPI_MASTER_RX_AUTOSUSP_DISABLE 0u
#define SPI_MASTER_KEEP_IO_STATE_DISABLE 0u
#define SPI_IO_SWAP_DISABLE 0u

#define GPIO_PIN_5 (1u << 5)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_MODE_AF_PP 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
#define GPIO_AF8_SPI6 8u

#define RCC_PERIPHCLK_SPI6 1u
#define RCC_SPI6CLKSOURCE_HSE 1u
#define BDMA_REQUEST_SPI6_TX 12u
#define DMA_MEMORY_TO_PERIPH 1u
#define DMA_PINC_DISABLE 0u
#define DMA_MINC_ENABLE 1u
#define DMA_PDATAALIGN_BYTE 0u
#define DMA_MDATAALIGN_BYTE 0u
#define DMA_NORMAL 0u
#define DMA_PRIORITY_LOW 0u
#define HAL_DMA_STATE_READY 1u
#define HAL_DMA_STATE_BUSY 2u
#define SPI_CR1_SPE 1u
#define SPI_CR1_CSTART 2u
#define SPI_CR1_CSUSP 4u
#define SPI_CFG1_TXDMAEN 1u
#define SPI_IT_EOT (1u << 0)
#define SPI_IT_TXP (1u << 1)
#define SPI_IT_RXP (1u << 2)
#define SPI_IT_DXP (1u << 3)
#define SPI_IT_UDR (1u << 4)
#define SPI_IT_OVR (1u << 5)
#define SPI_IT_FRE (1u << 6)
#define SPI_IT_MODF (1u << 7)
#define BDMA_CCR_EN 1u
#define BDMA_Channel0_IRQn 129
#define SPI6_IRQn 86
#define __HAL_RCC_SPI6_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_BDMA_CLK_ENABLE() ((void)0)
#define __DMB() ((void)0)
#define __HAL_DMA_DISABLE(handle) WS2812_TestDisableDMA(handle)
#define SET_BIT(reg, bit) ((reg) |= (bit))
#define __HAL_SPI_DISABLE_IT(handle, mask) ((handle)->Instance->IER &= ~(mask))
#define __HAL_LINKDMA(handle, field, dma) do { (handle)->field = &(dma); (dma).Parent = (handle); } while (0)
#define __HAL_DMA_GET_TC_FLAG_INDEX(dma) (1u)
#define __HAL_DMA_GET_HT_FLAG_INDEX(dma) (2u)
#define __HAL_DMA_GET_TE_FLAG_INDEX(dma) (4u)
#define __HAL_DMA_CLEAR_FLAG(dma, flags) ((void)(dma), (void)(flags))

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *handle, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *handle);
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *handle);
void WS2812_TestDisableDMA(DMA_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *handle);
uint32_t HAL_DMA_GetState(DMA_HandleTypeDef *handle);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *handle);
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *handle);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_ClearPendingIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
uint32_t HAL_GetTick(void);
uint32_t __get_IPSR(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);

#endif
