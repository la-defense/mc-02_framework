#ifndef MC02_HOST_BMI088_FAKE_BSP_GPIO_H
#define MC02_HOST_BMI088_FAKE_BSP_GPIO_H

#include <stdint.h>

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef enum { GPIO_EXTI_MODE_NONE = 0 } GPIO_EXTI_MODE_e;
typedef struct gpio_ins_temp GPIOInstance;
struct gpio_ins_temp
{
    GPIO_PinState pin_state;
    void (*gpio_model_callback)(GPIOInstance *);
    void *id;
};
typedef struct
{
    GPIO_PinState pin_state;
    GPIO_EXTI_MODE_e exti_mode;
    void (*gpio_model_callback)(GPIOInstance *);
    void *id;
} GPIO_Init_Config_s;

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config);
void GPIOSet(GPIOInstance *instance);
void GPIOReset(GPIOInstance *instance);

#endif
