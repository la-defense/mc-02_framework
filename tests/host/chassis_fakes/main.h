#ifndef MC02_HOST_CHASSIS_MAIN_H
#define MC02_HOST_CHASSIS_MAIN_H
#include <stdint.h>
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart1;
uint32_t HAL_GetTick(void);
static uint32_t mc02_chassis_test_primask;
#define __get_PRIMASK() (mc02_chassis_test_primask)
#define __disable_irq() (mc02_chassis_test_primask = 1u)
#define __set_PRIMASK(value) (mc02_chassis_test_primask = (value))
#endif
