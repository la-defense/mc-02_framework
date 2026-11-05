/** @file exti_group_test.c
 * @brief 使用 HAL 待处理位模型检查生产 EXTI 组处理函数。
 */
#include "main.h"
#include "test_check.h"

void EXTI15_10_IRQHandler(void);
static uint16_t pending;
static uint16_t callbacks[2];
static unsigned callback_count;

void HAL_GPIO_EXTI_IRQHandler(uint16_t pin)
{
    // 模拟 HAL 先判断待处理位，再清位并把原参数交给回调。
    if ((pending & pin) != 0u)
    {
        pending = (uint16_t)(pending & (uint16_t)~pin);
        TEST_CHECK(callback_count < 2u);
        callbacks[callback_count++] = pin;
    }
}

int main(void)
{
    // 两路同时到达时分别回调；其他引脚的待处理位不受影响。
    pending = GPIO_PIN_10 | GPIO_PIN_12 | (UINT16_C(1) << 11u);
    EXTI15_10_IRQHandler();
    TEST_CHECK(callback_count == 2u);
    TEST_CHECK(callbacks[0] == GPIO_PIN_10);
    TEST_CHECK(callbacks[1] == GPIO_PIN_12);
    TEST_CHECK(pending == (UINT16_C(1) << 11u));
    // 再次进入时不重复交付已经清除的事件。
    EXTI15_10_IRQHandler();
    TEST_CHECK(callback_count == 2u);
    // 单独触发一路时只交付该路。
    callback_count = 0u;
    pending = GPIO_PIN_12;
    EXTI15_10_IRQHandler();
    TEST_CHECK(callback_count == 1u && callbacks[0] == GPIO_PIN_12);
    return 0;
}
