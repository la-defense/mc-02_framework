/** @file adc.h
 * @brief ADC 板级启动测试使用的最小 HAL 声明。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t ClockPrescaler, Resolution, ScanConvMode, EOCSelection, LowPowerAutoWait, ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, ExternalTrigConv, ExternalTrigConvEdge, ConversionDataManagement, Overrun, LeftBitShift, OversamplingMode; } ADC_InitTypeDef;
typedef struct { uint32_t Request, Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, FIFOMode; } DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; DMA_HandleTypeDef *DMA_Handle; uint32_t State; } ADC_HandleTypeDef;
typedef struct { uint32_t Mode; } ADC_MultiModeTypeDef;
typedef struct { uint32_t Channel, Rank, SamplingTime, SingleDiff, OffsetNumber, Offset, OffsetSignedSaturation; } ADC_ChannelConfTypeDef;
typedef struct { uint32_t Pin, Mode, Pull; } GPIO_InitTypeDef;
typedef struct { uint32_t PLL2M, PLL2N, PLL2P, PLL2Q, PLL2R, PLL2RGE, PLL2VCOSEL, PLL2FRACN; } FakePll;
typedef struct { uint32_t PeriphClockSelection, AdcClockSelection; FakePll PLL2; } RCC_PeriphCLKInitTypeDef;
extern unsigned char fake_adc, fake_dma;
#define ADC1 (&fake_adc)
#define DMA1_Stream0 (&fake_dma)
#define GPIOA ((void *)1u)
#define GPIOC ((void *)2u)
#define HAL_OK 0u
#define HAL_ERROR 1u
#define HAL_ADC_STATE_RESET 0u
#define HAL_ADC_STATE_READY 1u
#define ENABLE 1u
#define DISABLE 0u
#define ADC_CHANNEL_4 4u
#define ADC_CHANNEL_19 19u
#define ADC_REGULAR_RANK_1 1u
#define ADC_REGULAR_RANK_2 2u
#define GPIO_PIN_4 (1u << 4)
#define GPIO_PIN_5 (1u << 5)
#define __HAL_LINKDMA(handle, field, dma) ((handle)->field = &(dma))
#define __HAL_RCC_ADC12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC12_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define DMA1_Stream0_IRQn 0
#define ADC_CLOCK_ASYNC_DIV64 0u
#define ADC_RESOLUTION_16B 0u
#define ADC_SCAN_ENABLE 0u
#define ADC_EOC_SINGLE_CONV 0u
#define ADC_SOFTWARE_START 0u
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0u
#define ADC_CONVERSIONDATA_DMA_CIRCULAR 0u
#define ADC_OVR_DATA_PRESERVED 0u
#define ADC_LEFTBITSHIFT_NONE 0u
#define ADC_MODE_INDEPENDENT 0u
#define ADC_SAMPLETIME_32CYCLES_5 0u
#define ADC_SINGLE_ENDED 0u
#define ADC_OFFSET_NONE 0u
#define RCC_PERIPHCLK_ADC 0u
#define RCC_PLL2VCIRANGE_3 0u
#define RCC_PLL2VCOWIDE 0u
#define RCC_ADCCLKSOURCE_PLL2 0u
#define GPIO_MODE_ANALOG 0u
#define GPIO_NOPULL 0u
#define DMA_REQUEST_ADC1 0u
#define DMA_PERIPH_TO_MEMORY 0u
#define DMA_PINC_DISABLE 0u
#define DMA_MINC_ENABLE 0u
#define DMA_PDATAALIGN_HALFWORD 0u
#define DMA_MDATAALIGN_HALFWORD 0u
#define DMA_CIRCULAR 0u
#define DMA_PRIORITY_LOW 0u
#define DMA_FIFOMODE_DISABLE 0u
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *dma);
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *dma);
uint32_t HAL_ADC_Init(ADC_HandleTypeDef *handle);
uint32_t HAL_ADC_DeInit(ADC_HandleTypeDef *handle);
uint32_t HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *handle, ADC_MultiModeTypeDef *config);
uint32_t HAL_ADC_ConfigChannel(ADC_HandleTypeDef *handle, ADC_ChannelConfTypeDef *config);
void HAL_ADC_MspInit(ADC_HandleTypeDef *handle);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *handle);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
void HAL_GPIO_DeInit(void *port, uint32_t pins);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void Error_Handler(void);
extern ADC_HandleTypeDef hadc1;
uint8_t MX_ADC1_InitChecked(void);
