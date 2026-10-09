#ifndef MC02_CMD_MAIN_H
#define MC02_CMD_MAIN_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart5, huart9;
uint32_t HAL_GetTick(void);
#endif
