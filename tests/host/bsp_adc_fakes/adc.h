#ifndef MC02_HOST_ADC_FAKE_ADC_H
#define MC02_HOST_ADC_FAKE_ADC_H

#include "main.h"

typedef struct
{
    void *Instance;
    volatile uint32_t ErrorCode;
} ADC_HandleTypeDef;

extern ADC_HandleTypeDef hadc1;

#define ADC1 ((void *)(uintptr_t)1u)
#define ADC_CALIB_OFFSET 0u
#define ADC_SINGLE_ENDED 0u

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t calibration_mode,
                                             uint32_t single_diff);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *data, uint32_t length);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

#endif
