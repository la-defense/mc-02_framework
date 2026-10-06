#ifndef MC02_HOST_HEATER_FAKE_ADC_H
#define MC02_HOST_HEATER_FAKE_ADC_H

#include "../../../bsp/adc/bsp_adc.h"

BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample);
float BSP_ADCGetVccIn(void);
#endif
