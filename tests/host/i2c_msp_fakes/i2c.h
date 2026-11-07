/** @file i2c.h
 * @brief 主机测试使用的 I2C2 与 DMA 最小 HAL 替身。
 */
#pragma once
#include <stdint.h>
typedef struct { uint32_t Timing, OwnAddress1, AddressingMode, DualAddressMode, OwnAddress2, OwnAddress2Masks, GeneralCallMode, NoStretchMode; } I2C_InitTypeDef;
typedef struct { uint32_t Request, Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, FIFOMode; } DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { void *Instance; I2C_InitTypeDef Init; DMA_HandleTypeDef *hdmarx, *hdmatx; } I2C_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t PeriphClockSelection, I2c123ClockSelection; } RCC_PeriphCLKInitTypeDef;
extern unsigned char fake_i2c, fake_dma_rx, fake_dma_tx;
extern I2C_HandleTypeDef hi2c2;
#define I2C2 (&fake_i2c)
#define DMA1_Stream2 (&fake_dma_rx)
#define DMA1_Stream3 (&fake_dma_tx)
#define GPIOB ((void *)1u)
#define HAL_OK 0u
#define I2C_ADDRESSINGMODE_7BIT 0u
#define I2C_DUALADDRESS_DISABLE 0u
#define I2C_OA2_NOMASK 0u
#define I2C_GENERALCALL_DISABLE 0u
#define I2C_NOSTRETCH_DISABLE 0u
#define I2C_ANALOGFILTER_ENABLE 0u
#define RCC_PERIPHCLK_I2C2 0u
#define RCC_I2C1235CLKSOURCE_D2PCLK1 0u
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_MODE_AF_OD 0u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_AF4_I2C2 0u
#define DMA_REQUEST_I2C2_RX 0u
#define DMA_REQUEST_I2C2_TX 0u
#define DMA_PERIPH_TO_MEMORY 0u
#define DMA_MEMORY_TO_PERIPH 0u
#define DMA_PINC_DISABLE 0u
#define DMA_MINC_ENABLE 0u
#define DMA_PDATAALIGN_BYTE 0u
#define DMA_MDATAALIGN_BYTE 0u
#define DMA_NORMAL 0u
#define DMA_PRIORITY_LOW 0u
#define DMA_FIFOMODE_DISABLE 0u
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_I2C2_CLK_DISABLE() ((void)0)
#define __HAL_LINKDMA(handle, field, dma) ((handle)->field = &(dma))
enum { I2C2_EV_IRQn, I2C2_ER_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, OTHER_IRQn, TEST_IRQ_COUNT };
uint32_t HAL_I2C_Init(I2C_HandleTypeDef *handle);
uint32_t HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *handle, uint32_t config);
uint32_t HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *handle, uint32_t config);
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *dma);
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *dma);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
void HAL_GPIO_DeInit(void *port, uint32_t pins);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void HAL_I2C_MspInit(I2C_HandleTypeDef *handle);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *handle);
void Error_Handler(void);
