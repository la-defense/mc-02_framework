/** @file stm32h7xx_ll_adc.h
 * @brief ADC 校准测试模拟硬件使能、启动与完成状态。
 */
#pragma once
#include <stdint.h>
uint32_t LL_ADC_IsEnabled(void *adc);
uint32_t LL_ADC_IsCalibrationOnGoing(void *adc);
void LL_ADC_StartCalibration(void *adc, uint32_t mode, uint32_t single);
