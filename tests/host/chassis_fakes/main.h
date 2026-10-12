#ifndef MC02_HOST_CHASSIS_MAIN_H
#define MC02_HOST_CHASSIS_MAIN_H
#include <stdint.h>
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart1;
#endif
