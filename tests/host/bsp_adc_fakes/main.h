#ifndef MC02_HOST_ADC_FAKE_MAIN_H
#define MC02_HOST_ADC_FAKE_MAIN_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *address, int32_t size);
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size);

#endif
