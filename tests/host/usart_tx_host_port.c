/** @file usart_tx_host_port.c
 * @brief 既有接收测试使用的UART硬件模型；队列及接收生产实现保持真实。
 * @note 专项测试另外链接H723硬件接入，核对寄存器和IRQ条件。
 */
#include "usart_tx_private.h"
#ifdef USART_TX_THREADS
#include <pthread.h>
static pthread_mutex_t tx_lock = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local uint32_t lock_depth;
#endif
uint32_t USARTTxEnter(void)
{
#ifdef USART_TX_THREADS
    uint32_t previous = lock_depth++;
    if (!previous)
        pthread_mutex_lock(&tx_lock);
    return previous;
#else
    return 0u;
#endif
}
void USARTTxExit(uint32_t saved)
{
#ifdef USART_TX_THREADS
    lock_depth = saved;
    if (!saved)
        pthread_mutex_unlock(&tx_lock);
#else
    (void)saved;
#endif
}
uint8_t USARTTxTaskContext(void)
{
    return 1u;
}
uint8_t USARTTxConfigured(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    return handle != NULL && (mode == USART_TRANSFER_DMA || mode == USART_TRANSFER_IT);
}
HAL_StatusTypeDef USARTTxStart(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length,
                               USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef result = mode == USART_TRANSFER_DMA ? HAL_UART_Transmit_DMA(handle, data, length)
                                                          : HAL_UART_Transmit_IT(handle, data, length);
    if (result == HAL_OK)
        handle->gState = 0x21u;
    return result;
}
uint8_t USARTTxStopped(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    (void)mode;
    return handle->gState == HAL_UART_STATE_READY;
}
HAL_StatusTypeDef USARTTxAbort(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    (void)mode;
#ifdef USART_TX_ABORT_MODEL
    extern HAL_StatusTypeDef UARTTestAbort(UART_HandleTypeDef * handle);
    return UARTTestAbort(handle);
#else
    handle->gState = HAL_UART_STATE_READY;
    return HAL_OK;
#endif
}
void USARTTxClear(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    (void)handle;
    (void)mode;
}
#ifndef USART_TX_LEASE_MODEL
uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    return 1u;
}
void BSP_HALTickReleaseBlockingWait(void)
{
}
#endif
