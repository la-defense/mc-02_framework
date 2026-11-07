/** @file main.h
 * @brief 为 UART 中断映射测试提供独立的 IRQ 编号和 HAL 声明。
 */
#pragma once
#include <stdint.h>
typedef enum {
    USART1_IRQn, USART2_IRQn, USART3_IRQn, UART5_IRQn, UART7_IRQn, UART8_IRQn,
    UART9_IRQn, USART10_IRQn, DMA1_Stream1_IRQn, DMA1_Stream4_IRQn,
    DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn, DMA2_Stream0_IRQn,
    DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn,
    DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn, TEST_IRQ_COUNT
} IRQn_Type;
void HAL_NVIC_DisableIRQ(IRQn_Type irq);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq);
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
