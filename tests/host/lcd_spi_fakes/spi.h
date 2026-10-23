#ifndef MC02_LCD_SPI_FAKE_H
#define MC02_LCD_SPI_FAKE_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_SPI_STATE_RESET, HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY_TX, HAL_SPI_STATE_ERROR } HAL_SPI_StateTypeDef;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpiob, fake_gpioc, fake_gpiod, fake_gpioe;
#define GPIOB (&fake_gpiob)
#define GPIOC (&fake_gpioc)
#define GPIOD (&fake_gpiod)
#define GPIOE (&fake_gpioe)
#define SPI1 ((void *)1)
#define SPI2 ((void *)2)
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
             BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial,
             NSSPMode, NSSPolarity, FifoThreshold, TxCRCInitializationPattern,
             RxCRCInitializationPattern, MasterSSIdleness, MasterInterDataIdleness,
             MasterReceiverAutoSusp, MasterKeepIOState, IOSwap;
} SPI_InitTypeDef;
typedef struct { void *Instance; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t PeriphClockSelection, Spi123ClockSelection; } RCC_PeriphCLKInitTypeDef;
extern SPI_HandleTypeDef hspi1, hspi2;
#define SPI_POLARITY_LOW 0u
#define SPI_POLARITY_HIGH 1u
#define SPI_PHASE_1EDGE 0u
#define SPI_PHASE_2EDGE 1u
#define SPI_FIRSTBIT_MSB 0u
#define SPI_MODE_MASTER 1u
#define SPI_DIRECTION_2LINES_TXONLY 2u
#define SPI_DIRECTION_2LINES 3u
#define SPI_DATASIZE_8BIT 8u
#define SPI_NSS_SOFT 0u
#define SPI_BAUDRATEPRESCALER_4 4u
#define SPI_BAUDRATEPRESCALER_32 32u
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
#define SPI_MASTER_KEEP_IO_STATE_ENABLE 1u
#define SPI_IO_SWAP_DISABLE 0u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define GPIO_PIN_1 (1u << 1)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_PIN_3 (1u << 3)
#define GPIO_PIN_7 (1u << 7)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_15 (1u << 15)
#define GPIO_MODE_AF_PP 2u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
#define GPIO_AF5_SPI1 5u
#define GPIO_AF5_SPI2 5u
#define RCC_PERIPHCLK_SPI1 1u
#define RCC_PERIPHCLK_SPI2 2u
#define RCC_SPI123CLKSOURCE_PLL 0u
#define __HAL_RCC_SPI1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI1_CLK_DISABLE() ((void)0)
#define __HAL_RCC_SPI2_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
uint32_t HAL_GetTick(void);
uint32_t __get_IPSR(void);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_DeInit(GPIO_TypeDef *, uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, uint32_t);
void HAL_Delay(uint32_t);
void Error_Handler(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
#endif
