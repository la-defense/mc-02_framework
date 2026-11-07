/** @file dma.h
 * @brief 全局 DMA 启动测试使用的时钟和 NVIC 替身。
 */
#pragma once
#include <stdint.h>
enum { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
       DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
       DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
       DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn,
       TEST_DMA_IRQ_COUNT };
void FakeDmaClockEnable(unsigned controller);
#define __HAL_RCC_DMA1_CLK_ENABLE() FakeDmaClockEnable(1u)
#define __HAL_RCC_DMA2_CLK_ENABLE() FakeDmaClockEnable(2u)
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub);
void HAL_NVIC_EnableIRQ(int irq);
void MX_DMA_Init(void);
