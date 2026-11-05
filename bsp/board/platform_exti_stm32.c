/** @file platform_exti_stm32.c
 * @brief 按单个引脚交给 HAL 处理共享 EXTI 组。
 */
#include "main.h"

/** @brief 处理板载 IMU 所在 EXTI 组的两个引脚。
 * @note HAL 对每个参数检查并清除各自的待处理位；同步采样配置不启用该 IRQ。
 */
void EXTI15_10_IRQHandler(void)
{
    // HAL 回调以单个引脚作为编号，因此不能将两个引脚合并传入。
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
}
