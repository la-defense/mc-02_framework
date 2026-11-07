/** @file i2c_msp_test.c
 * @brief 检查生产 I2C2 MSP 对中断、DMA 及重复退出的处理。
 */
#include "i2c.h"
#include "test_check.h"
unsigned char fake_i2c, fake_dma_rx, fake_dma_tx;
static unsigned enabled[TEST_IRQ_COUNT], pending[TEST_IRQ_COUNT], priorities[TEST_IRQ_COUNT];
static unsigned dma_init, dma_deinit, gpio_init, gpio_deinit, clock_calls;
uint32_t HAL_I2C_Init(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
uint32_t HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
uint32_t HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
uint32_t HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c) { (void)c; ++clock_calls; return HAL_OK; }
uint32_t HAL_DMA_Init(DMA_HandleTypeDef *d) { TEST_CHECK(d && d->Instance); ++dma_init; return HAL_OK; }
uint32_t HAL_DMA_DeInit(DMA_HandleTypeDef *d)
{
    TEST_CHECK(d && d->Instance); // 只释放成功绑定的 DMA。
    TEST_CHECK(!enabled[I2C2_EV_IRQn] && !enabled[I2C2_ER_IRQn] &&
               !enabled[DMA1_Stream2_IRQn] && !enabled[DMA1_Stream3_IRQn]); // 清理前必须停用全部相关中断。
    ++dma_deinit; return HAL_OK;
}
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *c) { TEST_CHECK(p == GPIOB && c); ++gpio_init; }
void HAL_GPIO_DeInit(void *p, uint32_t pins) { TEST_CHECK(p == GPIOB && pins); ++gpio_deinit; }
void HAL_NVIC_SetPriority(int irq, uint32_t p, uint32_t s) { TEST_CHECK(p == 5u && s == 0u); ++priorities[irq]; }
void HAL_NVIC_EnableIRQ(int irq)
{ TEST_CHECK(!pending[irq] && priorities[irq] && dma_init >= 2u && (dma_init % 2u) == 0u); enabled[irq] = 1u; }
void HAL_NVIC_DisableIRQ(int irq) { enabled[irq] = 0u; }
void HAL_NVIC_ClearPendingIRQ(int irq) { TEST_CHECK(!enabled[irq]); pending[irq] = 0u; }
void Error_Handler(void) { TEST_CHECK(0); }

int main(void)
{
    hi2c2.Instance = I2C2; // 使用生产句柄绑定的板级总线。
    enabled[OTHER_IRQn] = 1u; // 其他外设的中断必须保持独立。
    HAL_I2C_MspInit(0); HAL_I2C_MspDeInit(0); // 空句柄不能操作资源。
    I2C_HandleTypeDef other = {0}; // 未绑定的实例不能改变 I2C2 资源。
    other.Instance = (void *)1u;
    HAL_I2C_MspInit(&other); HAL_I2C_MspDeInit(&other);
    HAL_I2C_MspDeInit(&hi2c2); // 尚未初始化时不能释放共享引脚。
    TEST_CHECK(!gpio_deinit && !dma_deinit && enabled[OTHER_IRQn]);
    for (unsigned i = 0u; i < OTHER_IRQn; ++i) pending[i] = 1u; // 模拟旧的待处理事件。
    HAL_I2C_MspInit(&hi2c2);
    TEST_CHECK(clock_calls == 1u && dma_init == 2u && gpio_init == 1u);
    for (unsigned i = 0u; i < OTHER_IRQn; ++i) TEST_CHECK(enabled[i] && !pending[i]);
    HAL_I2C_MspInit(&hi2c2); // 重复初始化不重配已经使用的资源。
    TEST_CHECK(clock_calls == 1u && dma_init == 2u && gpio_init == 1u);
    for (unsigned i = 0u; i < OTHER_IRQn; ++i) pending[i] = 1u;
    HAL_I2C_MspDeInit(&hi2c2);
    TEST_CHECK(dma_deinit == 2u && gpio_deinit == 2u && enabled[OTHER_IRQn]);
    for (unsigned i = 0u; i < OTHER_IRQn; ++i) TEST_CHECK(!enabled[i] && !pending[i]);
    HAL_I2C_MspDeInit(&hi2c2); // 第二次退出不重新释放引脚或 DMA。
    TEST_CHECK(dma_deinit == 2u && gpio_deinit == 2u && enabled[OTHER_IRQn]);
    HAL_I2C_MspInit(&hi2c2); // 退出后允许重新初始化并恢复中断。
    TEST_CHECK(dma_init == 4u && gpio_init == 2u && enabled[I2C2_ER_IRQn]);
    return 0;
}
