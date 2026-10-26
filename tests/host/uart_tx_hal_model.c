/** @file uart_tx_hal_model.c
 * @brief 只替代UART/DMA/IRQ和时钟，队列、注册、协议及H723接入检查使用生产代码。
 */
#include "uart_tx_hal_model.h"
#include "test_check.h"
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
USART_TypeDef uart_registers[8];
DMA_Stream_TypeDef dma_registers[16];
UART_HandleTypeDef uart_handles[8];
DMA_HandleTypeDef uart_tx_dma[8], uart_rx_dma[8];
_Thread_local uint32_t mc02_test_ipsr, mc02_test_primask, mc02_test_basepri, mc02_test_faultmask;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static _Thread_local uint32_t critical_stack[16], critical_depth;
_Atomic uint32_t uart_model_ms, uart_model_calls, uart_model_aborts, uart_model_leases;
HAL_StatusTypeDef uart_model_start_result = HAL_OK, uart_model_abort_result = HAL_OK;
uint8_t *uart_model_rx[8], *uart_model_tx[8];
uint16_t uart_model_tx_length[8];
uint8_t uart_model_irq_enabled[32], uart_model_lease_denied;
static _Atomic unsigned allocation_calls, free_calls;
static _Atomic uint8_t fail_next_allocation;
static unsigned allocation_baseline, free_baseline;
static uint8_t check_allocations;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void __real_free(void *pointer);
static uint8_t UARTModelConsumeAllocationFailure(void)
{
    // Release优化可能把分配后清零合并为calloc，因此两种分配入口共用故障标记。
    return atomic_exchange(&fail_next_allocation, 0u);
}
void *__wrap_malloc(size_t size)
{
    atomic_fetch_add(&allocation_calls, 1u);
    if (UARTModelConsumeAllocationFailure()) return NULL;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    atomic_fetch_add(&allocation_calls, 1u);
    if (UARTModelConsumeAllocationFailure()) return NULL;
    return __real_calloc(count, size);
}
void __wrap_free(void *pointer)
{
    atomic_fetch_add(&free_calls, 1u);
    __real_free(pointer);
}
void UARTModelCheckRuntimeAllocation(void)
{
    allocation_baseline = allocation_calls;
    free_baseline = free_calls;
    check_allocations = 1u;
}
void UARTModelGetAllocationCounts(uint32_t *allocations, uint32_t *frees)
{
    if (allocations != NULL) *allocations = atomic_load(&allocation_calls);
    if (frees != NULL) *frees = atomic_load(&free_calls);
}
void UARTModelFailNextAllocation(void)
{
    atomic_store(&fail_next_allocation, 1u);
}
__attribute__((destructor)) static void CheckNoRuntimeAllocation(void)
{
    if (check_allocations)
        TEST_CHECK(allocation_calls == allocation_baseline && free_calls == free_baseline);
}
void mc02_test_disable_irq(void)
{
    if (!mc02_test_primask)
        pthread_mutex_lock(&mutex);
    mc02_test_primask = 1u;
}
void mc02_test_set_primask(uint32_t value)
{
    uint32_t previous = mc02_test_primask;
    mc02_test_primask = value;
    if (previous && !value)
        pthread_mutex_unlock(&mutex);
}
void UARTModelCriticalEnter(void)
{
    TEST_CHECK(critical_depth < 16u);
    critical_stack[critical_depth++] = mc02_test_primask;
    mc02_test_disable_irq();
}
void UARTModelCriticalExit(void)
{
    TEST_CHECK(critical_depth > 0u);
    mc02_test_set_primask(critical_stack[--critical_depth]);
}
uint32_t NVIC_GetEnableIRQ(IRQn_Type irq)
{
    return irq >= 0 && irq < 32 ? uart_model_irq_enabled[irq] : 0u;
}
void NVIC_ClearPendingIRQ(IRQn_Type irq)
{
    TEST_CHECK(irq >= 0 && irq < 18);
}
void mc02_dma_clear(DMA_HandleTypeDef *dma, uint32_t flags)
{
    (void)dma;
    (void)flags;
}
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *dma)
{
    return dma->State;
}
uint32_t HAL_GetTick(void)
{
    return atomic_load(&uart_model_ms);
}
uint8_t BSP_HALTickTryAcquireBlockingWait(void)
{
    if (uart_model_lease_denied)
        return 0u;
    atomic_fetch_add(&uart_model_leases, 1u);
    return 1u;
}
void BSP_HALTickReleaseBlockingWait(void)
{
    TEST_CHECK(atomic_fetch_sub(&uart_model_leases, 1u) > 0u);
}
void UARTModelInit(void)
{
    memset(uart_model_irq_enabled, 1, sizeof(uart_model_irq_enabled));
    for (unsigned p = 0u; p < 8u; ++p)
    {
        uart_registers[p].CR1 = USART_CR1_UE | USART_CR1_TE;
        uart_handles[p] = (UART_HandleTypeDef){.Instance = &uart_registers[p],
                                               .Init = {UART_MODE_TX, 8u, 0u},
                                               .gState = HAL_UART_STATE_READY,
                                               .hdmatx = &uart_tx_dma[p],
                                               .hdmarx = &uart_rx_dma[p]};
        uart_tx_dma[p] =
            (DMA_HandleTypeDef){.Instance = &dma_registers[p],
                                .Parent = &uart_handles[p],
                                .Init = {DMA_MEMORY_TO_PERIPH, DMA_NORMAL, DMA_MINC_ENABLE, DMA_PINC_DISABLE,
                                         DMA_MDATAALIGN_BYTE, DMA_PDATAALIGN_BYTE},
                                .State = HAL_DMA_STATE_READY};
        uart_rx_dma[p] = (DMA_HandleTypeDef){.Instance = &dma_registers[p + 8u], .Parent = &uart_handles[p]};
    }
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t length)
{
    (void)length;
    unsigned p = (unsigned)(h - uart_handles);
    TEST_CHECK(p < 8u);
    uart_model_rx[p] = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *data, uint16_t length)
{
    atomic_fetch_add(&uart_model_calls, 1u);
    if (uart_model_start_result != HAL_OK)
        return uart_model_start_result;
    unsigned p = (unsigned)(h - uart_handles);
    TEST_CHECK(p < 8u);
    uart_model_tx[p] = data;
    uart_model_tx_length[p] = length;
    h->gState = 0x21u;
    h->Instance->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef result = HAL_UART_Transmit_IT(h, data, length);
    if (result == HAL_OK)
    {
        h->Instance->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        h->Instance->CR3 |= USART_CR3_DMAT;
        h->hdmatx->State = HAL_DMA_STATE_BUSY;
        h->hdmatx->Instance->CR |= DMA_SxCR_EN;
    }
    return result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *h)
{
    TEST_CHECK(mc02_test_ipsr == 0u && mc02_test_primask == 0u);
    atomic_fetch_add(&uart_model_aborts, 1u);
    if (uart_model_abort_result != HAL_OK)
        return uart_model_abort_result;
    UARTModelCriticalEnter();
    h->Instance->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TXEIE_TXFNFIE);
    if (h->Instance->CR3 & USART_CR3_DMAT)
    {
        h->hdmatx->Instance->CR &= ~DMA_SxCR_EN;
        h->hdmatx->State = HAL_DMA_STATE_READY;
    }
    h->Instance->CR3 &= ~(USART_CR3_DMAT | USART_CR3_TXFTIE);
    h->gState = HAL_UART_STATE_READY;
    UARTModelCriticalExit();
    return HAL_OK;
}
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *dma)
{
    UARTModelCriticalEnter();
    dma->Instance->CR &= ~DMA_SxCR_EN;
    dma->State = HAL_DMA_STATE_READY;
    UARTModelCriticalExit();
    return HAL_OK;
}
void UARTModelDMAComplete(unsigned p)
{
    UARTModelCriticalEnter();
    uart_handles[p].hdmatx->State = HAL_DMA_STATE_READY;
    uart_handles[p].hdmatx->Instance->CR &= ~DMA_SxCR_EN;
    uart_handles[p].Instance->CR3 &= ~USART_CR3_DMAT;
    uart_handles[p].Instance->CR1 |= USART_CR1_TCIE;
    UARTModelCriticalExit();
}
void UARTModelComplete(unsigned p)
{
    UARTModelCriticalEnter();
    uart_handles[p].Instance->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TXEIE_TXFNFIE);
    uart_handles[p].gState = HAL_UART_STATE_READY;
    mc02_test_ipsr = 1u;
    HAL_UART_TxCpltCallback(&uart_handles[p]);
    mc02_test_ipsr = 0u;
    UARTModelCriticalExit();
}
