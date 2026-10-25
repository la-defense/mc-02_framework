#ifndef INIT_DIAGNOSTIC_TEST_MAIN_H
#define INIT_DIAGNOSTIC_TEST_MAIN_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
uint32_t HAL_GetTick(void);
#endif
