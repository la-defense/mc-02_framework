/** @file usart_tx_port.c
 * @brief H723 UART发送的HAL接入；只清理TX标志，保留共享UART的接收中断。
 */
#include "usart_tx_private.h"

uint32_t USARTTxEnter(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
void USARTTxExit(uint32_t saved)
{
    __set_PRIMASK(saved);
}
uint8_t USARTTxTaskContext(void)
{
    return __get_IPSR() == 0u && __get_PRIMASK() == 0u && __get_BASEPRI() == 0u && __get_FAULTMASK() == 0u;
}
static IRQn_Type TxUARTIRQ(const UART_HandleTypeDef *handle)
{
    USART_TypeDef *const devices[] = {USART1, USART2, USART3, UART5, UART7, UART8, UART9, USART10};
    const IRQn_Type irqs[] = {USART1_IRQn, USART2_IRQn, USART3_IRQn, UART5_IRQn,
                              UART7_IRQn,  UART8_IRQn,  UART9_IRQn,  USART10_IRQn};
    if (handle != NULL)
        for (unsigned i = 0u; i < DEVICE_USART_CNT; ++i)
            if (handle->Instance == devices[i])
                return irqs[i];
    return (IRQn_Type)-1;
}
static IRQn_Type TxDMAIRQ(const DMA_HandleTypeDef *dma)
{
    DMA_Stream_TypeDef *const streams[] = {DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
                                           DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
                                           DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
                                           DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7};
    const IRQn_Type irqs[] = {DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
                              DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn,
                              DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
                              DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn};
    if (dma != NULL)
        for (unsigned i = 0u; i < sizeof(streams) / sizeof(streams[0]); ++i)
            if (dma->Instance == streams[i])
                return irqs[i];
    return (IRQn_Type)-1;
}
uint8_t USARTTxConfigured(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    IRQn_Type irq = TxUARTIRQ(handle);
    if (irq < 0 || !NVIC_GetEnableIRQ(irq) || !(handle->Init.Mode & UART_MODE_TX) ||
        (handle->Instance->CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE) ||
        handle->Init.WordLength != UART_WORDLENGTH_8B || handle->Init.Parity != UART_PARITY_NONE)
        return 0u;
    if (mode == USART_TRANSFER_IT)
        return 1u;
    if (mode != USART_TRANSFER_DMA)
        return 0u;
    DMA_HandleTypeDef *dma = handle->hdmatx;
    irq = TxDMAIRQ(dma);
    return irq >= 0 && NVIC_GetEnableIRQ(irq) && dma->Parent == handle && dma->Init.Mode == DMA_NORMAL &&
           dma->Init.Direction == DMA_MEMORY_TO_PERIPH && dma->Init.MemInc == DMA_MINC_ENABLE &&
           dma->Init.PeriphInc == DMA_PINC_DISABLE && dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE &&
           dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}
HAL_StatusTypeDef USARTTxStart(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length,
                               USART_TRANSFER_MODE mode)
{
    return mode == USART_TRANSFER_DMA ? HAL_UART_Transmit_DMA(handle, data, length)
                                      : HAL_UART_Transmit_IT(handle, data, length);
}
uint8_t USARTTxStopped(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    if (handle->gState != HAL_UART_STATE_READY ||
        (handle->Instance->CR1 & (USART_CR1_TCIE | USART_CR1_TXEIE_TXFNFIE)) ||
        (handle->Instance->CR3 & (USART_CR3_DMAT | USART_CR3_TXFTIE)))
        return 0u;
    return mode != USART_TRANSFER_DMA ||
           (handle->hdmatx != NULL && HAL_DMA_GetState(handle->hdmatx) == HAL_DMA_STATE_READY &&
            (((DMA_Stream_TypeDef *)handle->hdmatx->Instance)->CR & DMA_SxCR_EN) == 0u);
}
HAL_StatusTypeDef USARTTxAbort(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    HAL_StatusTypeDef result = HAL_UART_AbortTransmit(handle);
    // HAL错误路径可能先清DMAT，AbortTransmit因而略过DMA；仍须单独停止当前TX stream。
    if (result == HAL_OK && mode == USART_TRANSFER_DMA && handle->hdmatx != NULL &&
        (HAL_DMA_GetState(handle->hdmatx) == HAL_DMA_STATE_BUSY ||
         (((DMA_Stream_TypeDef *)handle->hdmatx->Instance)->CR & DMA_SxCR_EN) != 0u))
        result = HAL_DMA_Abort(handle->hdmatx);
    return result;
}
void USARTTxClear(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode)
{
    __HAL_UART_CLEAR_FLAG(handle, UART_CLEAR_TCF);
    // UART IRQ同时负责接收，不能整体清掉其pending位；只清本发送DMA的标志和pending。
    if (mode == USART_TRANSFER_DMA)
    {
        DMA_HandleTypeDef *dma = handle->hdmatx;
        __HAL_DMA_CLEAR_FLAG(dma, __HAL_DMA_GET_TC_FLAG_INDEX(dma) | __HAL_DMA_GET_HT_FLAG_INDEX(dma) |
                                      __HAL_DMA_GET_TE_FLAG_INDEX(dma) | __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                                      __HAL_DMA_GET_FE_FLAG_INDEX(dma));
        NVIC_ClearPendingIRQ(TxDMAIRQ(dma));
    }
}
