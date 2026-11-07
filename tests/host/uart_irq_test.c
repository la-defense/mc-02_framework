/** @file uart_irq_test.c
 * @brief 用 NVIC 替身验证真实板级 UART／DMA 中断启停与隔离。
 */
#include "platform_uart_irq_stm32.h"
#include "main.h"
#include "test_check.h"
#include <string.h>

static unsigned enabled[TEST_IRQ_COUNT]; // 记录各中断目前是否允许响应。
static unsigned pending[TEST_IRQ_COUNT]; // 模拟退出前留下的待处理中断。
static unsigned priority_calls[TEST_IRQ_COUNT]; // 检查每个启用的中断都配置优先级。
void HAL_NVIC_DisableIRQ(IRQn_Type irq) { enabled[irq] = 0u; }
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq)
{
    TEST_CHECK(!enabled[irq]); // 清除待处理状态前必须先屏蔽该中断。
    pending[irq] = 0u;
}
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
    TEST_CHECK(preempt == 5u && sub == 0u); // 保持原有 RTOS 中断优先级。
    ++priority_calls[irq];
}
void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
    TEST_CHECK(!pending[irq] && priority_calls[irq]); // 旧事件清除并配置优先级后才启用。
    enabled[irq] = 1u;
}

int main(void)
{
    const struct { PlatformUartPort_t port; IRQn_Type uart, rx, tx; unsigned dma_count; } cases[] = {
        {PLATFORM_UART_1, USART1_IRQn, DMA2_Stream3_IRQn, DMA2_Stream2_IRQn, 2u}, // USART1 收发。
        {PLATFORM_UART_2, USART2_IRQn, DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, 2u}, // USART2 收发。
        {PLATFORM_UART_3, USART3_IRQn, DMA1_Stream6_IRQn, DMA1_Stream1_IRQn, 2u}, // USART3 收发。
        {PLATFORM_UART_5, UART5_IRQn, DMA1_Stream7_IRQn, DMA1_Stream7_IRQn, 1u}, // UART5 仅接收。
        {PLATFORM_UART_7, UART7_IRQn, UART7_IRQn, UART7_IRQn, 0u}, // UART7 无 DMA。
        {PLATFORM_UART_8, UART8_IRQn, DMA2_Stream7_IRQn, DMA2_Stream6_IRQn, 2u}, // UART8 收发。
        {PLATFORM_UART_9, UART9_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, 2u}, // UART9 收发。
        {PLATFORM_UART_10, USART10_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, 2u}, // USART10 收发。
    };
    // 依次启用各串口，并验证旧待处理事件在启用前被清除。
    for (unsigned i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        pending[cases[i].uart] = 1u; // 模拟串口上次留下的事件。
        if (cases[i].dma_count) pending[cases[i].rx] = 1u; // 模拟接收 DMA 旧事件。
        if (cases[i].dma_count > 1u) pending[cases[i].tx] = 1u; // 模拟发送 DMA 旧事件。
        TEST_CHECK(PlatformUartInterruptEnable(cases[i].port));
        TEST_CHECK(enabled[cases[i].uart]);
        if (cases[i].dma_count) TEST_CHECK(enabled[cases[i].rx]);
        if (cases[i].dma_count > 1u) TEST_CHECK(enabled[cases[i].tx]);
    }
    // 退出每个串口时，只允许修改其自身中断，并允许重复退出。
    for (unsigned i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        unsigned old[TEST_IRQ_COUNT]; // 保存其他串口的启用状态。
        memcpy(old, enabled, sizeof(old));
        pending[cases[i].uart] = 1u;
        if (cases[i].dma_count) pending[cases[i].rx] = 1u;
        if (cases[i].dma_count > 1u) pending[cases[i].tx] = 1u;
        TEST_CHECK(PlatformUartInterruptDisable(cases[i].port));
        TEST_CHECK(PlatformUartInterruptDisable(cases[i].port));
        TEST_CHECK(!enabled[cases[i].uart] && !pending[cases[i].uart]);
        if (cases[i].dma_count) TEST_CHECK(!enabled[cases[i].rx] && !pending[cases[i].rx]);
        if (cases[i].dma_count > 1u) TEST_CHECK(!enabled[cases[i].tx] && !pending[cases[i].tx]);
        for (unsigned n = 0u; n < TEST_IRQ_COUNT; ++n)
            if (n != (unsigned)cases[i].uart && (!cases[i].dma_count || n != (unsigned)cases[i].rx) &&
                (cases[i].dma_count < 2u || n != (unsigned)cases[i].tx))
                TEST_CHECK(enabled[n] == old[n]); // 其他串口状态保持原样。
    }
    TEST_CHECK(!PlatformUartInterruptEnable(PLATFORM_UART_INVALID));
    TEST_CHECK(!PlatformUartInterruptDisable(PLATFORM_UART_INVALID));
    return 0;
}
