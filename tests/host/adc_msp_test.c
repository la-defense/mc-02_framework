/** @file adc_msp_test.c
 * @brief 对生产 ADC 初始化注入时钟、DMA、HAL 和通道失败。
 */
#include "adc.h"
#include "test_check.h"
#include <string.h>
unsigned char fake_adc, fake_dma;
static unsigned failure, clocks, dma_init, dma_deinit, hal_init, channel_calls, irq_enable, irq_disable;
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c) { (void)c; ++clocks; return failure == 1u ? HAL_ERROR : HAL_OK; }
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *d) { TEST_CHECK(d && d->Instance); ++dma_init; return failure == 2u ? HAL_ERROR : HAL_OK; }
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *d) { if (d) { TEST_CHECK(d->Instance); ++dma_deinit; } return HAL_OK; }
uint32_t HAL_ADC_Init(ADC_HandleTypeDef *h)
{
    ++hal_init;
    HAL_ADC_MspInit(h); // 模拟 HAL 的第二次 MSP 调用。
    if (failure == 3u) return HAL_ERROR;
    h->State = HAL_ADC_STATE_READY;
    return HAL_OK;
}
uint32_t HAL_ADC_DeInit(ADC_HandleTypeDef *h) { HAL_ADC_MspDeInit(h); h->State = HAL_ADC_STATE_RESET; return HAL_OK; }
uint32_t HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *h, ADC_MultiModeTypeDef *c) { (void)h; (void)c; return failure == 4u ? HAL_ERROR : HAL_OK; }
uint32_t HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{ (void)h; ++channel_calls; TEST_CHECK(c->Rank == channel_calls); return failure == 4u + channel_calls ? HAL_ERROR : HAL_OK; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *c) { (void)p; (void)c; }
void HAL_GPIO_DeInit(void *p, uint32_t pins) { (void)p; (void)pins; }
void HAL_NVIC_SetPriority(int irq, uint32_t p, uint32_t s) { (void)irq; TEST_CHECK(p == 5u && s == 0u); }
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; ++irq_enable; }
void HAL_NVIC_DisableIRQ(int irq) { (void)irq; ++irq_disable; }
void HAL_NVIC_ClearPendingIRQ(int irq) { (void)irq; }
void Error_Handler(void) { TEST_CHECK(0); } // 受检查入口不能触发致命处理。

int main(void)
{
    for (failure = 1u; failure <= 6u; ++failure)
    {
        memset(&hadc1, 0, sizeof(hadc1)); // 每轮从未使用的 ADC 开始。
        clocks = dma_init = dma_deinit = hal_init = channel_calls = irq_enable = irq_disable = 0u;
        TEST_CHECK(!MX_ADC1_InitChecked() && hadc1.Instance == NULL);
        TEST_CHECK(clocks == 1u); // 时钟准备不会被 HAL 再次执行。
        if (failure == 1u) TEST_CHECK(!dma_init && !hal_init && !irq_enable);
        else TEST_CHECK(dma_init == 1u && dma_deinit == 1u && irq_disable >= 1u);
        if (failure == 2u) TEST_CHECK(!hal_init && !irq_enable); // DMA 失败后不访问 ADC。
    }
    failure = 0u;
    memset(&hadc1, 0, sizeof(hadc1));
    clocks = dma_init = dma_deinit = hal_init = channel_calls = irq_enable = irq_disable = 0u;
    TEST_CHECK(MX_ADC1_InitChecked());
    TEST_CHECK(clocks == 1u && dma_init == 1u && hal_init == 1u && channel_calls == 2u);
    TEST_CHECK(!MX_ADC1_InitChecked() && hadc1.Instance == ADC1); // 不覆盖已使用的 ADC。
    TEST_CHECK(HAL_ADC_DeInit(&hadc1) == HAL_OK);
    return 0;
}
