/** @file platform_uart_irq_stm32.c
 * @brief 将逻辑串口映射到 MC-02 的 UART 和 DMA 中断。
 */
#include "platform_uart_irq_stm32.h"
#include "main.h"

typedef struct {
    IRQn_Type uart; // 串口自身中断。
    IRQn_Type dma_rx; // 接收 DMA 中断；无 DMA 时不使用。
    IRQn_Type dma_tx; // 发送 DMA 中断；无 DMA 时不使用。
    unsigned char dma_count; // 0：无 DMA；1：仅接收；2：收发都有。
} UartInterrupts;

// 根据板级接线选择本串口独占的中断，避免影响其他设备。
static unsigned char UartInterruptsFor(PlatformUartPort_t port, UartInterrupts *out)
{
    switch (port)
    {
    case PLATFORM_UART_1: *out = (UartInterrupts){USART1_IRQn, DMA2_Stream3_IRQn, DMA2_Stream2_IRQn, 2u}; break; // USART1 收发。
    case PLATFORM_UART_2: *out = (UartInterrupts){USART2_IRQn, DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, 2u}; break; // USART2 收发。
    case PLATFORM_UART_3: *out = (UartInterrupts){USART3_IRQn, DMA1_Stream6_IRQn, DMA1_Stream1_IRQn, 2u}; break; // USART3 收发。
    case PLATFORM_UART_5: *out = (UartInterrupts){UART5_IRQn, DMA1_Stream7_IRQn, DMA1_Stream7_IRQn, 1u}; break; // UART5 仅接收。
    case PLATFORM_UART_7: *out = (UartInterrupts){UART7_IRQn, UART7_IRQn, UART7_IRQn, 0u}; break; // UART7 无 DMA。
    case PLATFORM_UART_8: *out = (UartInterrupts){UART8_IRQn, DMA2_Stream7_IRQn, DMA2_Stream6_IRQn, 2u}; break; // UART8 收发。
    case PLATFORM_UART_9: *out = (UartInterrupts){UART9_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, 2u}; break; // UART9 收发。
    case PLATFORM_UART_10: *out = (UartInterrupts){USART10_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, 2u}; break; // USART10 收发。
    default: return 0u; // 未绑定编号不触碰任何中断。
    }
    return 1u; // 调用者可使用已填写的中断集合。
}

// 清除旧的待处理中断后按 FreeRTOS 允许的优先级启用。
static void UartInterruptEnableOne(IRQn_Type irq)
{
    HAL_NVIC_DisableIRQ(irq); // 配置期间不执行旧处理函数。
    HAL_NVIC_ClearPendingIRQ(irq); // 避免上次退出时的事件被误处理。
    HAL_NVIC_SetPriority(irq, 5u, 0u); // 沿用工程现有中断优先级。
    HAL_NVIC_EnableIRQ(irq); // 资源准备后才允许响应。
}

// 先屏蔽中断再清除待处理状态，防止释放资源后进入处理函数。
static void UartInterruptDisableOne(IRQn_Type irq)
{
    HAL_NVIC_DisableIRQ(irq); // 停止新的中断响应。
    HAL_NVIC_ClearPendingIRQ(irq); // 清除已进入 NVIC 的旧事件。
}

unsigned char PlatformUartInterruptEnable(PlatformUartPort_t port)
{
    UartInterrupts irqs; // 只在编号有效后读取映射。
    if (!UartInterruptsFor(port, &irqs)) return 0u; // 无效编号不操作硬件。
    if (irqs.dma_count > 0u) UartInterruptEnableOne(irqs.dma_rx); // 先准备接收 DMA。
    if (irqs.dma_count > 1u) UartInterruptEnableOne(irqs.dma_tx); // 再准备发送 DMA。
    UartInterruptEnableOne(irqs.uart); // 最后允许串口中断。
    return 1u; // 本串口所需中断均已配置。
}

unsigned char PlatformUartInterruptDisable(PlatformUartPort_t port)
{
    UartInterrupts irqs; // 保存本次退出涉及的中断。
    if (!UartInterruptsFor(port, &irqs)) return 0u; // 不修改其他串口。
    UartInterruptDisableOne(irqs.uart); // 先禁止串口产生新的处理。
    if (irqs.dma_count > 0u) UartInterruptDisableOne(irqs.dma_rx); // 停止接收 DMA 中断。
    if (irqs.dma_count > 1u) UartInterruptDisableOne(irqs.dma_tx); // 停止发送 DMA 中断。
    return 1u; // 此时可以继续释放该串口的资源。
}
