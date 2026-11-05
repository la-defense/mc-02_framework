/** @file gpio.h
 * @brief GPIO 启动测试所需的板级引脚和 HAL 最小声明。
 */
#ifndef MC02_TEST_POWER_GPIO_H
#define MC02_TEST_POWER_GPIO_H
#include <stdint.h>
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIOA ((GPIO_TypeDef *)(uintptr_t)1u)
#define GPIOB ((GPIO_TypeDef *)(uintptr_t)2u)
#define GPIOC ((GPIO_TypeDef *)(uintptr_t)3u)
#define GPIOD ((GPIO_TypeDef *)(uintptr_t)4u)
#define GPIOE ((GPIO_TypeDef *)(uintptr_t)5u)
#define GPIO_PIN_8 (UINT32_C(1) << 8u)
#define GPIO_PIN_10 (UINT32_C(1) << 10u)
#define GPIO_PIN_11 (UINT32_C(1) << 11u)
#define GPIO_PIN_12 (UINT32_C(1) << 12u)
#define GPIO_PIN_13 (UINT32_C(1) << 13u)
#define GPIO_PIN_14 (UINT32_C(1) << 14u)
#define GPIO_PIN_15 (UINT32_C(1) << 15u)
#define POWER_24V_2_Pin GPIO_PIN_13
#define POWER_24V_1_Pin GPIO_PIN_14
#define POWER_5V_Pin GPIO_PIN_15
#define CS2_ACCEL_Pin (UINT32_C(1) << 0u)
#define CS2_GYRO_Pin (UINT32_C(1) << 3u)
#define DCMI_PWDN_Pin (UINT32_C(1) << 5u)
#define ACC_INT_Pin GPIO_PIN_10
#define GYRO_INT_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_15
#define DCMI_REST_GPIO_Port GPIOB
#define DCMI_REST_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_BLK_Pin GPIO_PIN_10
#define LCD_RES_Pin GPIO_PIN_11
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_MODE_IT_FALLING 2u
#define GPIO_MODE_AF_PP 3u
#define GPIO_MODE_INPUT 4u
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_SPEED_FREQ_VERY_HIGH 1u
#define GPIO_AF0_MCO 0u
#define EXTI15_10_IRQn 40
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, uint32_t state);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_NVIC_DisableIRQ(int irq);
void MX_GPIO_Init(void);
#endif
