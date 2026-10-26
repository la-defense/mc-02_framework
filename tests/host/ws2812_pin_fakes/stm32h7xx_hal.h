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

extern GPIO_TypeDef fake_gpioa;
#define GPIOA (&fake_gpioa)
#define SPI6 ((void *)6)

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
    void *Instance;
    SPI_InitTypeDef Init;
    HAL_SPI_StateTypeDef State;
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
#define __HAL_RCC_SPI6_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *handle, uint8_t *data,
                                   uint16_t size, uint32_t timeout);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);

#endif
