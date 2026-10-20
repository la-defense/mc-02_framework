/** @file gpio.h
 * @brief 按端口和引脚分别记录片选电平，供 SPI 主机测试使用。
 */
#ifndef MC02_HOST_SPI_FAKE_GPIO_H
#define MC02_HOST_SPI_FAKE_GPIO_H

#include <stdint.h>

typedef struct { uint16_t pins; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);

#endif
