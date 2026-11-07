/** @file dma_init_test.c
 * @brief 检查全局 DMA 准备不会提前启用未注册外设的中断。
 */
#include "dma.h"
#include "test_check.h"
static unsigned clock_mask, priority_count, enable_count;
void FakeDmaClockEnable(unsigned controller) { clock_mask |= 1u << controller; }
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t sub)
{
    TEST_CHECK(irq >= 0 && irq < TEST_DMA_IRQ_COUNT && priority == 5u && sub == 0u);
    ++priority_count; // 保留 CubeMX 中各通道的优先级设置。
}
void HAL_NVIC_EnableIRQ(int irq) { (void)irq; ++enable_count; }
int main(void)
{
    MX_DMA_Init(); // 运行生产入口，外设各自完成后才启用中断。
    TEST_CHECK(clock_mask == ((1u << 1u) | (1u << 2u)));
    TEST_CHECK(priority_count == TEST_DMA_IRQ_COUNT && enable_count == 0u);
    return 0;
}
