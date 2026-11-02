/** @file platform_uart_tx_stm32.c
 * @brief 将逻辑串口发送操作映射到原有有界队列。
 */
#include "../usart/platform_uart.h"
#include "board_uart_stm32.h"
#include "bsp_usart.h"

USARTSendStatus_t PlatformUartSubmit(PlatformUartInstance *instance, const USARTTxRequest_t *request, uint32_t *id)
{
    return USARTSubmitTx((USARTInstance *)instance, request, id);
}

USARTSendStatus_t PlatformUartCancel(PlatformUartInstance *instance, uint32_t id)
{
    return USARTCancelTx((USARTInstance *)instance, id);
}

USARTSendStatus_t PlatformUartGetTxState(const PlatformUartInstance *instance, USARTTxState_t *output)
{
    return USARTGetTxState((const USARTInstance *)instance, output);
}
