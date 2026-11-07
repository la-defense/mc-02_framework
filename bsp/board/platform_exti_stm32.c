/** @file platform_exti_stm32.c
 * @brief 按单个引脚交给 HAL 处理共享 EXTI 组。
 */
#include "main.h"

/** @brief 分别处理 EXTI 10 至 15 号线的待处理中断。
 * @note HAL 对每个参数检查并清除各自的待处理位；同步采样配置不启用该 IRQ。
 */
void EXTI15_10_IRQHandler(void)
{
    // 每次只传一根线，让 HAL 独立检查并清除对应的待处理位。
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}
