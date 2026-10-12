#ifndef MC02_VCP_FAKE_MAIN_H
#define MC02_VCP_FAKE_MAIN_H
#include <stdint.h>
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void __DMB(void);
#endif
