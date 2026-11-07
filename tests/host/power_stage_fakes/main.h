/** @file main.h
 * @brief 受控电源板级测试的 GPIO 寄存器替身。
 */
#ifndef MC02_POWER_STAGE_TEST_MAIN_H
#define MC02_POWER_STAGE_TEST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t MODER, ODR; } GPIO_TypeDef;
extern GPIO_TypeDef power_test_gpio;
#define GPIOC (&power_test_gpio)
#define POWER_24V_1_Pin (UINT16_C(1) << 14u)
#define POWER_24V_2_Pin (UINT16_C(1) << 13u)
#define POWER_5V_Pin (UINT16_C(1) << 15u)
#define GPIO_PIN_SET 1u
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t value);
#endif
