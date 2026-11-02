/** @file platform_uart_stm32.c
 * @brief 将逻辑串口及队列操作映射到 MC-02 的 STM32 实例。
 */
#include "../usart/platform_uart.h"
#include "board_uart_stm32.h"
#include "bsp_usart.h"
#include <stddef.h>

uint8_t PlatformUartIsBound(PlatformUartPort_t port)
{
    return BoardUartResolve(port) != NULL;
}

PlatformUartInstance *PlatformUartRegister(PlatformUartPort_t port, const PlatformUartConfig *config)
{
    UART_HandleTypeDef *handle = BoardUartResolve(port); // 只在平台后端解析真实串口。
    if (handle == NULL || config == NULL) return NULL; // 未绑定或无配置时不触碰注册表。
    USART_Init_Config_s native = {0}; // 初始化所有字段，避免传入未定义的回调。
    native.usart_handle = handle; // 交给原注册流程检查重复及启动 DMA。
    native.recv_buff_size = config->receive_capacity; // 保留原接收块容量。
    native.module_callback = config->receive; // 原样传递任务接收回调。
    native.context = config->context; // 回调上下文由业务长期保存。
    native.tx_guard = config->tx_guard; // 发送前仍执行原身份检查。
    native.tx_gate = config->tx_gate; // 发送队列继续使用原额度规则。
    native.tx_observer = config->tx_observer; // 终态仍先通知观察者。
    return USARTRegister(&native); // 隐藏底层指针，不复制或访问注册结果。
}

uint8_t PlatformUartRestartReceive(PlatformUartInstance *instance)
{
    return USARTServiceInit((USARTInstance *)instance);
}

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
