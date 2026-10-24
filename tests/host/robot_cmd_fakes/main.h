#ifndef MC02_CMD_MAIN_H
#define MC02_CMD_MAIN_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;
extern UART_HandleTypeDef huart5, huart9;
extern CAN_HandleTypeDef hcan1;
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
uint32_t HAL_GetTick(void);
#endif
