/** @file main.h
 * @brief UART专项的寄存器、DMA、IRQ模型；复用既有SPI测试的DMA类型。
 */
#ifndef MC02_UART_TX_FAKE_MAIN_H
#define MC02_UART_TX_FAKE_MAIN_H
#include "../bsp_spi_fakes/spi.h"
#include <stdatomic.h>
typedef struct
{
    uint32_t CR1, CR3;
} USART_TypeDef;
extern USART_TypeDef uart_registers[8];
typedef struct
{
    USART_TypeDef *Instance;
    struct
    {
        uint32_t Mode, WordLength, Parity;
    } Init;
    uint32_t gState;
    DMA_HandleTypeDef *hdmatx, *hdmarx;
} UART_HandleTypeDef;
#define USART1 (&uart_registers[0])
#define USART1_IRQn 18
#define USART2 (&uart_registers[1])
#define USART2_IRQn 19
#define USART3 (&uart_registers[2])
#define USART3_IRQn 20
#define UART5 (&uart_registers[3])
#define UART5_IRQn 21
#define UART7 (&uart_registers[4])
#define UART7_IRQn 22
#define UART8 (&uart_registers[5])
#define UART8_IRQn 23
#define UART9 (&uart_registers[6])
#define UART9_IRQn 24
#define USART10 (&uart_registers[7])
#define USART10_IRQn 25

#define UART_MODE_TX 1u
#define UART_WORDLENGTH_8B 8u
#define UART_PARITY_NONE 0u
#define HAL_UART_STATE_READY 0x20u
#define USART_CR1_TCIE 1u
#define USART_CR1_UE 0x10u
#define USART_CR1_TE 0x20u
#define USART_CR1_TXEIE_TXFNFIE 2u
#define USART_CR3_DMAT 1u
#define USART_CR3_TXFTIE 2u
#define UART_CLEAR_TCF 1u
#define DMA_IT_HT 1u
#define __HAL_DMA_DISABLE_IT(h, f) ((void)(h), (void)(f))
#define __HAL_UART_CLEAR_FLAG(h, f) ((void)(h), (void)(f))
#define __DMB() atomic_thread_fence(memory_order_seq_cst)
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void UARTModelCriticalEnter(void);
void UARTModelCriticalExit(void);
#endif
