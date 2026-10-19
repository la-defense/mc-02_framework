/**
 * @file bsp_gpio.h
 * @brief 提供 IST8310 初始化测试使用的 GPIO 类型和函数声明。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_IIC_TEST_BSP_GPIO_H
#define MC02_IIC_TEST_BSP_GPIO_H

#include <stdint.h>

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum {
    GPIO_EXTI_MODE_RISING = 0,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE
} GPIO_EXTI_MODE_e;

typedef struct {
    uintptr_t unused;
} GPIO_TypeDef;

typedef struct GPIOInstance {
    GPIO_TypeDef *GPIOx;
    GPIO_PinState pin_state;
    GPIO_EXTI_MODE_e exti_mode;
    uint16_t GPIO_Pin;
    void (*gpio_model_callback)(struct GPIOInstance *);
    void *id;
} GPIOInstance;

typedef struct {
    GPIO_TypeDef *GPIOx;
    GPIO_PinState pin_state;
    GPIO_EXTI_MODE_e exti_mode;
    uint16_t GPIO_Pin;
    void (*gpio_model_callback)(GPIOInstance *);
    void *id;
} GPIO_Init_Config_s;

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config);
void GPIOReset(GPIOInstance *instance);
void GPIOSet(GPIOInstance *instance);

#endif
