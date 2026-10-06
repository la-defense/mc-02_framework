#ifndef MC02_HOST_SPI_FAKE_GPIO_H
#define MC02_HOST_SPI_FAKE_GPIO_H

#include <stdint.h>

typedef struct { uint8_t unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);

#endif
