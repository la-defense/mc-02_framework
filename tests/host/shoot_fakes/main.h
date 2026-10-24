#ifndef MC02_SHOOT_FAKE_MAIN_H
#define MC02_SHOOT_FAKE_MAIN_H
#include <stdint.h>
typedef struct { uint32_t marker; } CAN_HandleTypeDef;
typedef struct { uint32_t marker; } UART_HandleTypeDef;
extern CAN_HandleTypeDef hcan2;
uint32_t HAL_GetTick(void);
#endif
