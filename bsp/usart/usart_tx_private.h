/** @file usart_tx_private.h
 * @brief UART发送内部硬件接入点；只供BSP实现和主机硬件模型使用。
 */
#ifndef MC02_USART_TX_PRIVATE_H
#define MC02_USART_TX_PRIVATE_H
#include "bsp_usart.h"
// 编号0留给“尚未提交”，无符号回绕后跳过0。
static inline uint32_t USARTTxNextId(uint32_t previous)
{
    return previous == UINT32_MAX ? 1u : previous + 1u;
}
// 注册表只在启动时改变；按指针查找再解引用，拒绝未知或接收已隔离的对象。
int USARTRegisteredIndex(const USARTInstance *instance);
USARTInstance *USARTRegisteredAt(unsigned index);
// 仅登记当前传输失败，不在ISR等待停止。
void USARTTxErrorISR(UART_HandleTypeDef *handle);
// 保存并恢复原PRIMASK；HAL等待和业务回调必须在退出后执行。
uint32_t USARTTxEnter(void);
void USARTTxExit(uint32_t saved);
// 提交、取消和推进只接受未屏蔽中断的普通任务。
uint8_t USARTTxTaskContext(void);
// 检查当前IT/DMA条件；只有返回1才可尝试非阻塞HAL启动。
uint8_t USARTTxConfigured(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode);
HAL_StatusTypeDef USARTTxStart(UART_HandleTypeDef *handle, uint8_t *data, uint16_t length,
                               USART_TRANSFER_MODE mode);
// 确认TX状态、请求位和DMA stream都已停止后，才允许清标志并复用数据。
uint8_t USARTTxStopped(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode);
HAL_StatusTypeDef USARTTxAbort(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode);
void USARTTxClear(UART_HandleTypeDef *handle, USART_TRANSFER_MODE mode);
#endif
