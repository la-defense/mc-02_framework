/** @file uart_msp_test.c
 * @brief 对生产 UART 初始化注入时钟、DMA、HAL 和 FIFO 失败。
 */
#include "usart.h"
#include "platform_uart_irq_stm32.h"
#include "test_check.h"
#include <string.h>

unsigned char fake_uart_devices[8], fake_dma_streams[16];
static unsigned fail_clock, fail_dma, fail_hal, fail_fifo;
static unsigned clock_calls, dma_calls, dma_deinit, hal_calls, fifo_calls, irq_enable, irq_disable;
static PlatformUartPort_t last_disabled; // 检查失败清理没有关闭其他串口。
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config)
{ (void)config; ++clock_calls; return fail_clock ? HAL_ERROR : HAL_OK; }
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *dma)
{ TEST_CHECK(dma && dma->Instance); ++dma_calls; return dma_calls == fail_dma ? HAL_ERROR : HAL_OK; }
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *dma)
{ if (dma) { TEST_CHECK(dma->Instance); ++dma_deinit; } return HAL_OK; }
uint32_t HAL_UART_Init(UART_HandleTypeDef *handle)
{
    ++hal_calls;
    HAL_UART_MspInit(handle); // 模拟 HAL 的第二次 MSP 调用。
    if (fail_hal) return HAL_ERROR;
    handle->gState = HAL_UART_STATE_READY;
    return HAL_OK;
}
uint32_t HAL_RS485Ex_Init(UART_HandleTypeDef *handle, uint32_t p, uint32_t a, uint32_t d)
{ (void)p; (void)a; (void)d; return HAL_UART_Init(handle); }
uint32_t HAL_UART_DeInit(UART_HandleTypeDef *handle)
{ HAL_UART_MspDeInit(handle); handle->gState = HAL_UART_STATE_RESET; return HAL_OK; }
static uint32_t FifoResult(void) { ++fifo_calls; return fifo_calls == fail_fifo ? HAL_ERROR : HAL_OK; }
uint32_t HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *h, uint32_t v) { (void)h; (void)v; return FifoResult(); }
uint32_t HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *h, uint32_t v) { (void)h; (void)v; return FifoResult(); }
uint32_t HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *h) { (void)h; return FifoResult(); }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
void HAL_GPIO_DeInit(void *port, uint32_t pins) { (void)port; (void)pins; }
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub) { (void)irq; (void)priority; (void)sub; }
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; }
unsigned char PlatformUartInterruptEnable(PlatformUartPort_t port) { (void)port; ++irq_enable; return 1u; }
unsigned char PlatformUartInterruptDisable(PlatformUartPort_t port) { last_disabled = port; ++irq_disable; return 1u; }
void Error_Handler(void) { TEST_CHECK(0); } // 受检查入口不能走致命处理。

static void ResetAttempt(UART_HandleTypeDef *handle)
{
    memset(handle, 0, sizeof(*handle)); // 各轮从未初始化的句柄开始。
    fail_clock = fail_dma = fail_hal = fail_fifo = 0u;
    clock_calls = dma_calls = dma_deinit = hal_calls = fifo_calls = irq_enable = irq_disable = 0u;
}

int main(void)
{
    const struct { UART_HandleTypeDef *handle; uint8_t (*start)(void); unsigned dma_count; } cases[] = {
        {&huart5, MX_UART5_InitChecked, 1u}, // UART5 只有接收 DMA。
        {&huart7, MX_UART7_InitChecked, 0u}, // UART7 没有 DMA。
        {&huart8, MX_UART8_InitChecked, 2u}, // UART8 收发 DMA。
        {&huart9, MX_UART9_InitChecked, 2u}, // UART9 收发 DMA。
        {&huart1, MX_USART1_UART_InitChecked, 2u}, // USART1 收发 DMA。
        {&huart2, MX_USART2_UART_InitChecked, 2u}, // USART2 使用 RS485 配置。
        {&huart3, MX_USART3_UART_InitChecked, 2u}, // USART3 使用 RS485 配置。
        {&huart10, MX_USART10_UART_InitChecked, 2u}, // USART10 诊断串口。
    };
    for (unsigned i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        ResetAttempt(cases[i].handle);
        fail_clock = 1u; // 时钟失败不能访问 UART 或配置 DMA。
        TEST_CHECK(!cases[i].start() && !cases[i].handle->Instance);
        TEST_CHECK(clock_calls == 1u && !dma_calls && !hal_calls && !irq_enable);
        for (unsigned step = 1u; step <= cases[i].dma_count; ++step)
        {
            ResetAttempt(cases[i].handle);
            fail_dma = step; // 逐一测试每路 DMA 部分失败后的清理。
            TEST_CHECK(!cases[i].start() && !cases[i].handle->Instance);
            TEST_CHECK(dma_calls == step && dma_deinit == step && !hal_calls && irq_disable == 1u);
        }
        for (unsigned step = 0u; step <= 3u; ++step)
        {
            ResetAttempt(cases[i].handle);
            fail_hal = step == 0u; // 分别模拟 HAL 初始化及三个 FIFO 步骤失败。
            fail_fifo = step;
            TEST_CHECK(!cases[i].start() && !cases[i].handle->Instance);
            TEST_CHECK(clock_calls == 1u && dma_calls == cases[i].dma_count);
            TEST_CHECK(dma_deinit == cases[i].dma_count && irq_enable == 1u && irq_disable == 1u);
        }
        ResetAttempt(cases[i].handle);
        TEST_CHECK(cases[i].start()); // 正常情况 HAL 内部不能重复分配 MSP。
        TEST_CHECK(clock_calls == 1u && dma_calls == cases[i].dma_count && hal_calls == 1u);
        void *original = cases[i].handle->Instance;
        TEST_CHECK(!cases[i].start() && cases[i].handle->Instance == original); // 不破坏在用句柄。
        TEST_CHECK(HAL_UART_DeInit(cases[i].handle) == HAL_OK); // 为下一轮释放测试资源。
    }
    ResetAttempt(&huart9);
    TEST_CHECK(MX_UART9_InitChecked()); // 先保持另一个端口在运行。
    void *other_instance = huart9.Instance;
    ResetAttempt(&huart8);
    fail_dma = 2u; // UART8 第二路 DMA 失败时只清理 UART8。
    TEST_CHECK(!MX_UART8_InitChecked());
    TEST_CHECK(last_disabled == PLATFORM_UART_8 && huart9.Instance == other_instance);
    TEST_CHECK(huart9.gState == HAL_UART_STATE_READY); // 运行中的 UART9 不受影响。
    TEST_CHECK(HAL_UART_DeInit(&huart9) == HAL_OK);
    return 0;
}
