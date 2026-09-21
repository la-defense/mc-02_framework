#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

void BSP_ADCInit(void);
float BSP_ADCGetVccIn(void);
uint16_t BSP_ADCGetRawVccIn(void);
uint16_t BSP_ADCGetRawKey(void);

#endif // !BSP_ADC_H
