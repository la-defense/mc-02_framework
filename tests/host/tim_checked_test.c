/** @file tim_checked_test.c
 * @brief 在生成的定时器初始化入口逐步注入失败，检查输出引脚和另一控制器。
 */
#include "tim.h"
#include "test_check.h"
#include <stddef.h>

GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioe;
static unsigned fail_step, gpio_inits;
extern TIM_HandleTypeDef htim3, htim12;
int HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer) { (void)timer; return fail_step == 1u ? 1 : HAL_OK; }
int HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *timer, TIM_MasterConfigTypeDef *config)
{ (void)timer; (void)config; return fail_step == 2u ? 1 : HAL_OK; }
int HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *config, uint32_t channel)
{ (void)timer; (void)config; (void)channel; return fail_step == 3u ? 1 : HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config)
{ (void)port; (void)config; ++gpio_inits; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t value)
{ (void)port; (void)pin; (void)value; }
void Error_Handler(void) { TEST_CHECK(0); } // 受检查入口不应进入致命处理。

int main(void)
{
    TEST_CHECK(MX_TIM12_InitChecked()); // 先准备蜂鸣器，用于检查加热失败的隔离。
    for (fail_step = 1u; fail_step <= 3u; ++fail_step)
    {
        const unsigned previous = gpio_inits; // 记录本轮之前的输出引脚配置数。
        TEST_CHECK(!MX_TIM3_InitChecked()); // 三个 HAL 步骤任一失败都要返回失败。
        TEST_CHECK(htim3.Instance == NULL && htim12.Instance == TIM12);
        TEST_CHECK(gpio_inits == previous); // 失败时不得将加热引脚交给 PWM。
    }
    fail_step = 0u;
    TEST_CHECK(MX_TIM3_InitChecked()); // 允许测试替身模拟新一次启动成功。
    for (fail_step = 1u; fail_step <= 3u; ++fail_step)
    {
        const unsigned previous = gpio_inits;
        TEST_CHECK(!MX_TIM12_InitChecked()); // 逐个检查蜂鸣器的失败路径。
        TEST_CHECK(htim12.Instance == NULL && htim3.Instance == TIM3);
        TEST_CHECK(gpio_inits == previous); // 失败时不配置蜂鸣器输出引脚。
    }
    return 0;
}
