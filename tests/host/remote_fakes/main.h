#ifndef REMOTE_TEST_MAIN_H
#define REMOTE_TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } DMA_HandleTypeDef;
typedef struct { uint32_t gState; DMA_HandleTypeDef *hdmarx; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart5, huart9;
#define HAL_UART_STATE_READY 0x20u
#define DMA_IT_HT 1u
#define __HAL_DMA_DISABLE_IT(h, flags) ((void)(h), (void)(flags))
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline void __DMB(void) {}
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
#endif
