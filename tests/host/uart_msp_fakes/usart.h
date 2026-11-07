/** @file usart.h
 * @brief 主机测试使用的 UART、DMA、GPIO 和时钟最小 HAL 声明。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling, OneBitSampling, ClockPrescaler; } UART_InitTypeDef;
typedef struct { uint32_t AdvFeatureInit; } UART_AdvFeatureInitTypeDef;
typedef struct { uint32_t Request, Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, FIFOMode; } DMA_InitTypeDef;
typedef struct { void *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; UART_AdvFeatureInitTypeDef AdvancedInit; DMA_HandleTypeDef *hdmarx, *hdmatx; uint32_t gState; } UART_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t PeriphClockSelection, Usart234578ClockSelection, Usart16ClockSelection; } RCC_PeriphCLKInitTypeDef;

extern unsigned char fake_uart_devices[8], fake_dma_streams[16];
#define UART5 (&fake_uart_devices[0])
#define UART7 (&fake_uart_devices[1])
#define UART8 (&fake_uart_devices[2])
#define UART9 (&fake_uart_devices[3])
#define USART1 (&fake_uart_devices[4])
#define USART2 (&fake_uart_devices[5])
#define USART3 (&fake_uart_devices[6])
#define USART10 (&fake_uart_devices[7])
#define DMA1_Stream1 (&fake_dma_streams[1])
#define DMA1_Stream4 (&fake_dma_streams[4])
#define DMA1_Stream5 (&fake_dma_streams[5])
#define DMA1_Stream6 (&fake_dma_streams[6])
#define DMA1_Stream7 (&fake_dma_streams[7])
#define DMA2_Stream0 (&fake_dma_streams[8])
#define DMA2_Stream1 (&fake_dma_streams[9])
#define DMA2_Stream2 (&fake_dma_streams[10])
#define DMA2_Stream3 (&fake_dma_streams[11])
#define DMA2_Stream4 (&fake_dma_streams[12])
#define DMA2_Stream5 (&fake_dma_streams[13])
#define DMA2_Stream6 (&fake_dma_streams[14])
#define DMA2_Stream7 (&fake_dma_streams[15])
#define GPIOA ((void *)1u)
#define GPIOB ((void *)2u)
#define GPIOC ((void *)3u)
#define GPIOD ((void *)4u)
#define GPIOE ((void *)5u)
#define HAL_OK 0u
#define HAL_ERROR 1u
#define HAL_UART_STATE_RESET 0u
#define HAL_UART_STATE_READY 1u
#define __HAL_LINKDMA(handle, field, dma) ((handle)->field = &(dma))

uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *dma);
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *dma);
uint32_t HAL_UART_Init(UART_HandleTypeDef *handle);
uint32_t HAL_RS485Ex_Init(UART_HandleTypeDef *handle, uint32_t polarity, uint32_t assertion, uint32_t deassertion);
uint32_t HAL_UART_DeInit(UART_HandleTypeDef *handle);
uint32_t HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *handle, uint32_t value);
uint32_t HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *handle, uint32_t value);
uint32_t HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *handle);
void HAL_UART_MspInit(UART_HandleTypeDef *handle);
void HAL_UART_MspDeInit(UART_HandleTypeDef *handle);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
void HAL_GPIO_DeInit(void *port, uint32_t pins);
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_DisableIRQ(int irq);
void Error_Handler(void);
extern UART_HandleTypeDef huart5, huart7, huart8, huart9, huart1, huart2, huart3, huart10;
uint8_t MX_UART5_InitChecked(void);
uint8_t MX_UART7_InitChecked(void);
uint8_t MX_UART8_InitChecked(void);
uint8_t MX_UART9_InitChecked(void);
uint8_t MX_USART1_UART_InitChecked(void);
uint8_t MX_USART2_UART_InitChecked(void);
uint8_t MX_USART3_UART_InitChecked(void);
uint8_t MX_USART10_UART_InitChecked(void);
#define DMA_FIFOMODE_DISABLE 0u // 主机测试只需要占位值。
#define DMA_MDATAALIGN_BYTE 0u // 主机测试只需要占位值。
#define DMA_MEMORY_TO_PERIPH 0u // 主机测试只需要占位值。
#define DMA_MINC_ENABLE 0u // 主机测试只需要占位值。
#define DMA_NORMAL 0u // 主机测试只需要占位值。
#define DMA_PDATAALIGN_BYTE 0u // 主机测试只需要占位值。
#define DMA_PERIPH_TO_MEMORY 0u // 主机测试只需要占位值。
#define DMA_PINC_DISABLE 0u // 主机测试只需要占位值。
#define DMA_PRIORITY_HIGH 0u // 主机测试只需要占位值。
#define DMA_PRIORITY_LOW 0u // 主机测试只需要占位值。
#define DMA_PRIORITY_VERY_HIGH 0u // 主机测试只需要占位值。
#define DMA_REQUEST_UART5_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_UART8_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_UART8_TX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_UART9_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_UART9_TX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART10_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART10_TX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART1_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART1_TX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART2_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART2_TX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART3_RX 0u // 主机测试只需要占位值。
#define DMA_REQUEST_USART3_TX 0u // 主机测试只需要占位值。
#define GPIO_AF11_UART9 0u // 主机测试只需要占位值。
#define GPIO_AF11_USART10 0u // 主机测试只需要占位值。
#define GPIO_AF4_USART10 0u // 主机测试只需要占位值。
#define GPIO_AF7_UART7 0u // 主机测试只需要占位值。
#define GPIO_AF7_USART1 0u // 主机测试只需要占位值。
#define GPIO_AF7_USART2 0u // 主机测试只需要占位值。
#define GPIO_AF7_USART3 0u // 主机测试只需要占位值。
#define GPIO_AF8_UART5 0u // 主机测试只需要占位值。
#define GPIO_AF8_UART8 0u // 主机测试只需要占位值。
#define GPIO_MODE_AF_PP 0u // 主机测试只需要占位值。
#define GPIO_NOPULL 0u // 主机测试只需要占位值。
#define GPIO_PIN_0 (1u << 0) // 主机测试只需要占位值。
#define GPIO_PIN_1 (1u << 1) // 主机测试只需要占位值。
#define GPIO_PIN_10 (1u << 10) // 主机测试只需要占位值。
#define GPIO_PIN_12 (1u << 12) // 主机测试只需要占位值。
#define GPIO_PIN_14 (1u << 14) // 主机测试只需要占位值。
#define GPIO_PIN_15 (1u << 15) // 主机测试只需要占位值。
#define GPIO_PIN_2 (1u << 2) // 主机测试只需要占位值。
#define GPIO_PIN_3 (1u << 3) // 主机测试只需要占位值。
#define GPIO_PIN_4 (1u << 4) // 主机测试只需要占位值。
#define GPIO_PIN_5 (1u << 5) // 主机测试只需要占位值。
#define GPIO_PIN_6 (1u << 6) // 主机测试只需要占位值。
#define GPIO_PIN_7 (1u << 7) // 主机测试只需要占位值。
#define GPIO_PIN_8 (1u << 8) // 主机测试只需要占位值。
#define GPIO_PIN_9 (1u << 9) // 主机测试只需要占位值。
#define GPIO_SPEED_FREQ_LOW 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_UART5 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_UART7 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_UART8 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_UART9 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_USART1 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_USART10 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_USART2 0u // 主机测试只需要占位值。
#define RCC_PERIPHCLK_USART3 0u // 主机测试只需要占位值。
#define RCC_USART16910CLKSOURCE_D2PCLK2 0u // 主机测试只需要占位值。
#define RCC_USART234578CLKSOURCE_D2PCLK1 0u // 主机测试只需要占位值。
#define UART_ADVFEATURE_NO_INIT 0u // 主机测试只需要占位值。
#define UART_DE_POLARITY_HIGH 0u // 主机测试只需要占位值。
#define UART_HWCONTROL_NONE 0u // 主机测试只需要占位值。
#define UART_MODE_RX 0u // 主机测试只需要占位值。
#define UART_MODE_TX_RX 0u // 主机测试只需要占位值。
#define UART_ONE_BIT_SAMPLE_DISABLE 0u // 主机测试只需要占位值。
#define UART_OVERSAMPLING_16 0u // 主机测试只需要占位值。
#define UART_PARITY_EVEN 0u // 主机测试只需要占位值。
#define UART_PARITY_NONE 0u // 主机测试只需要占位值。
#define UART_PRESCALER_DIV1 0u // 主机测试只需要占位值。
#define UART_RXFIFO_THRESHOLD_1_8 0u // 主机测试只需要占位值。
#define UART_STOPBITS_1 0u // 主机测试只需要占位值。
#define UART_TXFIFO_THRESHOLD_1_8 0u // 主机测试只需要占位值。
#define UART_WORDLENGTH_8B 0u // 主机测试只需要占位值。
#define UART_WORDLENGTH_9B 0u // 主机测试只需要占位值。
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART5_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART5_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART7_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART7_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART8_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART8_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART9_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_UART9_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART10_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART10_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART1_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART2_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART3_CLK_DISABLE() ((void)0) // 测试不访问硬件时钟。
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0) // 测试不访问硬件时钟。
#define UART5_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define UART7_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define UART8_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define UART9_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define USART10_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define USART1_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define USART2_IRQn 0 // NVIC 行为由单独的中断测试验证。
#define USART3_IRQn 0 // NVIC 行为由单独的中断测试验证。
