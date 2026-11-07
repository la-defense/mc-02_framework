/** @file main.h
 * @brief 为共享 EXTI 处理测试提供最小 HAL 声明。
 */
#ifndef MC02_EXTI_TEST_MAIN_H
#define MC02_EXTI_TEST_MAIN_H
#include <stdint.h>
#define GPIO_PIN_10 (UINT16_C(1) << 10u) /**< 测试中的加速度计中断。 */
#define GPIO_PIN_11 (UINT16_C(1) << 11u) /**< 同组的第 11 号中断线。 */
#define GPIO_PIN_12 (UINT16_C(1) << 12u) /**< 测试中的陀螺仪中断。 */
#define GPIO_PIN_13 (UINT16_C(1) << 13u) /**< 同组的第 13 号中断线。 */
#define GPIO_PIN_14 (UINT16_C(1) << 14u) /**< 同组的第 14 号中断线。 */
#define GPIO_PIN_15 (UINT16_C(1) << 15u) /**< 同组的第 15 号中断线。 */
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
#endif
