/** @file uart_tx_hal_model.h
 * @brief 声明可控制的UART硬件模型；每个测试进程初始化一次，不重置生产注册表。
 */
#ifndef MC02_UART_TX_HAL_MODEL_H
#define MC02_UART_TX_HAL_MODEL_H
#include "main.h"
extern UART_HandleTypeDef uart_handles[8];
extern DMA_HandleTypeDef uart_tx_dma[8], uart_rx_dma[8];
extern _Atomic uint32_t uart_model_ms, uart_model_calls, uart_model_aborts, uart_model_leases;
extern HAL_StatusTypeDef uart_model_start_result, uart_model_abort_result;
extern uint8_t *uart_model_rx[8], *uart_model_tx[8];
extern uint16_t uart_model_tx_length[8];
extern uint8_t uart_model_irq_enabled[32], uart_model_lease_denied;
void UARTModelInit(void);
void UARTModelCheckRuntimeAllocation(void);
void UARTModelDMAComplete(unsigned port);
void UARTModelComplete(unsigned port);
#endif
